//! Draw the Rust logo centered on a 90 degree rotated 128x64px display
//!
//! Image was created with ImageMagick:
//!
//! ```bash
//! convert rust.png -depth 1 gray:rust.raw
//! ```
//!
//! Run on a Blue Pill with `xargo run --target thumbv7m-none-eabi --example image_i2c --features graphics`

#![no_std]

extern crate cortex_m;
extern crate embedded_graphics;
extern crate embedded_hal as hal;
extern crate ssd1306;
extern crate stm32f103xx_hal as blue_pill;

use blue_pill::i2c::{DutyCycle, I2c, Mode};
use blue_pill::prelude::*;
use embedded_graphics::Drawing;
use ssd1306::Builder;
use ssd1306::displayrotation::DisplayRotation;
use embedded_graphics::image::{Image, Image1BPP};

fn main() {
    let dp = blue_pill::stm32f103xx::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);

    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

    let i2c = I2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 400_000,
            duty_cycle: DutyCycle::Ratio1to1,
        },
        clocks,
        &mut rcc.apb1,
    );

    let mut disp = Builder::new()
        // Set initial rotation at 90 degrees clockwise
        .with_rotation(DisplayRotation::Rotate90)
        .connect_i2c(i2c).into_egfx();

    disp.init().unwrap();
    disp.flush().unwrap();

    // Contrived example to test builder and instance methods. Sets rotation to 270 degress
    // or 90 degress counterclockwise
    disp.set_rotation(DisplayRotation::Rotate270).unwrap();

    let (w, h) = disp.get_dimensions();

    let im = Image1BPP::new(
        include_bytes!("./rust.raw"),
        64,
        64,
        (w as u32 / 2 - 64 / 2, h as u32 / 2 - 64 / 2),
    );

    disp.draw(im.into_iter());

    disp.flush().unwrap();
}