#![no_std]
#![no_main]

use core::cell::{Cell, RefCell};
use core::pin::Pin;
use panic_semihosting as _;

use cortex_m::delay::Delay;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, exception, ExceptionFrame};
use embedded_graphics::{
    pixelcolor::{BinaryColor, Bgr565, Rgb565, RgbColor},
    Drawable,
    prelude::*,
    geometry::{Size, Point},
};
use embedded_graphics::primitives::{Circle, PrimitiveStyleBuilder, StyledDrawable};
use embedded_plots::{
    axis::Scale,
    curve::{Curve, PlotPoint},
    single_plot::SinglePlot,
};

use stm32f1xx_hal::{pac, prelude::*, timer::Timer, adc, gpio, stm32};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use stm32f1xx_hal::{
    i2c::{BlockingI2c, DutyCycle, Mode},
    prelude::*,
};
use stm32f1xx_hal::gpio::Analog;
use stm32f1xx_hal::pac::{ADC1, GPIOB, interrupt, TIM2, TIM3};
use stm32f1xx_hal::timer::{CounterMs, CounterUs, Event};

type AdcPin = gpio::PB0<Analog>;


static G_TIM2: Mutex<RefCell<Option<CounterUs<TIM2>>>> = Mutex::new(RefCell::new(None));
static G_TIM3: Mutex<RefCell<Option<CounterMs<TIM3>>>> = Mutex::new(RefCell::new(None));
static DIS_FLAG: Mutex<Cell<bool>> = Mutex::new(Cell::new(true));
static ADC: Mutex<RefCell<Option<adc<ADC1>>>> = Mutex::new(RefCell::new(None));
static ADC_PIN: Mutex<RefCell<Option<AdcPin>>> = Mutex::new(RefCell::new(None));
static POINTS: Mutex<RefCell<[u8; 128]>> = Mutex::new(RefCell::new([32; 128]));
static INDEX: Mutex<Cell<usize>> = Mutex::new(Cell::new(0));

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();
    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut nvic = &mut cp.NVIC;
    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr
        .use_hse(8.MHz())
        .sysclk(64.MHz())
        .hclk(64.MHz())
        .pclk1(32.MHz())
        .pclk2(32.MHz())
        .adcclk(8.MHz())
        .freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain();

    let mut gpiob = dp.GPIOB.split();

    //init interrupts
    let mut time2 = dp.TIM2.counter_us(&clocks);
    let mut time3 = dp.TIM3.counter_ms(&clocks);
    time2.start(390.micros()).unwrap();
    time3.start(50.millis()).unwrap();
    time2.listen(Event::Update);
    time3.listen(Event::Update);
    unsafe {
        cortex_m::Peripherals::NVIC::unmask(interrupt::TIM2);
        cortex_m::Peripherals::NVIC::unmask(interrupt::TIM3);
    }
    cortex_m::peripheral::NVIC::unpend(stm32::Interrupt::TIM2);
    cortex_m::peripheral::NVIC::unpend(stm32::Interrupt::TIM2);

    let mut delay = Delay::new(cp.SYST, 64000000);

    let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

    let mut adc1 = adc::Adc::adc1(dp.ADC1, clocks);
    let mut adc_pin = gpiob.pb0.into_analog(&mut gpiob.crl);

    let i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 400_000.Hz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );
    let interface = I2CDisplayInterface::new(i2c);

    let mut display = Ssd1306::new
        (interface,
         DisplaySize128x64,
         DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().expect("failed to init display");
    let mut plot_point1 = PlotPoint { x : 0 , y : 32};
    let mut data: [PlotPoint; 128]= [PlotPoint{x:0,y:32}; 128];


    cortex_m::interrupt::free(|cs| {
        G_TIM2.borrow(cs).replace(Some(time2));
        G_TIM3.borrow(cs).replace(Some(time3));
        ADC.borrow(cs).replace(Some(adc1));
        ADC_PIN.borrow(cs).replace(Some(adc_pin));
    });

    loop {
        cortex_m::interrupt::free(|cs| {
            let mut flag = DIS_FLAG.borrow(cs);
            let mut points = POINTS.borrow(cs).borrow_mut();
            if flag.get() {
                let mut data: [PlotPoint; 128] = [PlotPoint { x: 0, y: 32 }; 128];
                for i in points.iter() {
                    data[*i as usize] = PlotPoint { x: *i as i32, y: points[*i as usize] as i32 }
                }

                let curve = Curve::from_data(&data);
                let plot = SinglePlot::new(
                    &curve,
                    Scale::RangeFraction(2),
                    Scale::RangeFraction(1)).into_drawable(Point { x: 0, y: 0 }, Point { x: 128, y: 64 }).set_color(BinaryColor::On);
                plot.draw(&mut display).expect("draw function failed!");
                display.flush().unwrap();
                flag.set(false);
            }
        });
    }
}

#[interrupt]
fn TIM2() {
    cortex_m::interrupt::free(|cs| {
        let mut points = POINTS.borrow(cs).borrow_mut();
        let mut index = INDEX.borrow(cs);
        let mut adc = ADC.borrow(cs).borrow_mut().unwrap();
        let mut adc_pin = ADC_PIN.borrow(cs).borrow_mut().unwrap();

        let value: u32 = adc.read(&mut adc_pin).unwrap();

        if index.get() <= 128 {
            points[index.get()] = (value / 64) as u8;
            index.set(index.get() + 1);
        } else {
            index.set(0);
        }


        let mut timer = G_TIM2.borrow(cs).borrow_mut();
        timer.as_mut().unwrap().clear_interrupt(Event::Update);
    });
}

#[interrupt]
fn TIM3() {
    cortex_m::interrupt::free(|cs| {
        let mut flag = DIS_FLAG.borrow(cs).set(true);
        let mut timer = G_TIM3.borrow(cs).borrow_mut();
        timer.as_mut().unwrap().clear_interrupt(Event::Update);
    });
}

