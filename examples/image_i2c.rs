//! Draw a 1 bit per pixel black and white image. On a 128x64 SSD1306 display over I2C.
//!
//! Image was created with ImageMagick:
//!
//! ```bash
//! convert rust.png -depth 1 gray:rust.raw
//! ```
//!
//! This example is for the STM32F103 "Blue Pill" board using I2C1.
//!
//! Wiring connections are as follows for a CRIUS-branded display:
//!
//! ```
//!      Display -> Blue Pill
//! (black)  GND -> GND
//! (red)    +5V -> VCC
//! (yellow) SDA -> PB9
//! (green)  SCL -> PB8
//! ```
//!
//! Run on a Blue Pill with `cargo run --example image_i2c`.

#![no_std]
#![no_main]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_semihosting;
extern crate stm32f1xx_hal as hal;

use cortex_m_rt::ExceptionFrame;
use cortex_m_rt::{entry, exception};
//use cortex_m_semihosting::hprintln;
use embedded_graphics::image::Image1BPP;
use embedded_graphics::prelude::*;
use hal::i2c::{BlockingI2c, DutyCycle, Mode};
use hal::prelude::*;
use hal::stm32;
use nb::block;
use ssd1306::prelude::*;
use ssd1306::Builder;
use stm32f1xx_hal::timer::Timer;

const PI: f32 = 3.14159;
const PI2: f32 = PI * PI;
const PI3: f32 = PI2 * PI;
const PI4: f32 = PI3 * PI;
const PI5: f32 = PI4 * PI;

fn _sin(x: f32) -> f32 {
    let x3 = x * x * x;
    let x5 = x3 * x * x;
    2.0 * PI * x - 4.0 * PI3 * x3 / 3.0 + 4.0 * PI5 * x5 / 15.0
}

fn sin(x: f32) -> f32 {
    if x >= 1.0 {
        sin(x - 1.0)
    } else if x >= 0.51 {
        -sin(x - 0.5)
    } else if x >= 0.26 {
        sin(0.5 - x)
    } else {
        _sin(x)
    }
}

fn cos(x: f32) -> f32 {
    sin(x + 0.25)
    // let x2 = x * x;
    // let x4 = x2 * x * x;
    // let x6 = x4 * x * x;
    // 1.0 - x2 / 2.0 + x4 / 24.0 - x6 / 720.0
}

const W8: u32 = 64 / 8;

fn get_px(xs: &[u8], x: u32, y: u32, w: u32, h: u32) -> u8 {
    if x >= w || y >= h {
        return 0;
    }
    let bit_n = 7 - x % 8;
    let coord = (y * W8 + (x / 8)) as usize;
    (xs[coord] & (1 << bit_n)) >> bit_n
}

fn set_px(xs: &mut [u8], v: u8, x: u32, y: u32, w: u32, h: u32) {
    let bit_n = 7 - x % 8;
    let coord = (y * W8 + (x / 8)) as usize;
    let v0 = xs[coord] & !(1 << bit_n);
    xs[coord] = v0 | (v << bit_n);
}

fn rotate(dst: &mut [u8], src: &[u8], w: u32, h: u32, rot: u8) {
    let theta = 1.0 - (rot as f32) / 255.0;
    let a = cos(theta);
    let b = -sin(theta);
    let c = -b;
    let d = a;
    let (w2, h2) = (w as f32 / 2.0, h as f32 / 2.0);
    for y in 0..h {
        let _y = y as f32 - h2;
        let _x0 = b * _y;
        let _y0 = d * _y;
        for x in 0..w {
            let _x = x as f32 - w2;
            let x0 = a * _x + _x0;
            let y0 = c * _x + _y0;
            let v = get_px(src, (x0 + w2) as u32, (y0 + w2) as u32, w, h);
            set_px(dst, v, x, y, w, h);
        }
    }
}

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    let cp = cortex_m::Peripherals::take().unwrap();
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

    let i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 400_000,
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        &mut rcc.apb1,
        1000,
        10,
        1000,
        1000,
    );

    let mut disp: GraphicsMode<_> = Builder::new().connect_i2c(i2c).into();

    disp.init().unwrap();
    disp.flush().unwrap();

    const H: u32 = 64;
    const W: u32 = 64;
    const W8: u32 = W / 8;
    let im_bytes = include_bytes!("./rust.raw");
    // let mut im_raw = vec![0; h * w];
    let mut im_raw: [u8; (H * W8) as usize] = [0; (H * W8) as usize];
    //hprintln!("im_bytes len: {}", im_bytes.len()).unwrap();
    //hprintln!("im_raw len: {}", im_raw.len()).unwrap();
    im_raw.copy_from_slice(im_bytes);
    let mut im_raw_rot: [u8; (H * W8) as usize] = [0; (H * W8) as usize];

    // Wait for the timer to trigger an update and change the state of the LED
    let mut timer = Timer::syst(cp.SYST, 8.hz(), clocks);
    let mut rot = 0;
    loop {
        //hprintln!("loop {}", rot).unwrap();
        block!(timer.wait()).unwrap();
        led.set_high();
        block!(timer.wait()).unwrap();
        led.set_low();
        rotate(&mut im_raw_rot, &im_raw, W, H, rot);
        let im = Image1BPP::new(&im_raw_rot[..], H, W);
        disp.draw(im.translate(Coord::new(32, 0)).into_iter());
        disp.flush().unwrap();
        rot += 8;
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
