use rp2040_hal as hal;
use hal::i2c::I2C;
use hal::pac::I2C0;
use embedded_hal::blocking::i2c::{Write, WriteRead};

const LSM6DS33_ADDR: u8 = 0x6B;
const WHO_AM_I: u8 = 0x0F;
const CTRL1_XL: u8 = 0x10;
const CTRL2_G: u8 = 0x11;
const OUTX_L_G: u8 = 0x22;
const OUTX_L_XL: u8 = 0x28;

pub struct Imu<P> {
    i2c: I2C<I2C0, P>,
}

impl<P> Imu<P> 
{
    pub fn new(i2c: I2C<I2C0, P>) -> Self {
        Self { i2c }
    }

    pub fn init(&mut self) -> Result<(), ()> {
        let mut id = [0u8; 1];
        self.i2c.write_read(LSM6DS33_ADDR, &[WHO_AM_I], &mut id).map_err(|_| ())?;
        
        if id[0] != 0x69 {
            return Err(());
        }

        // Enable Accel: 1.66 kHz, 2g, 400Hz BW
        self.i2c.write(LSM6DS33_ADDR, &[CTRL1_XL, 0x80]).map_err(|_| ())?;

        // Enable Gyro: 1.66 kHz, 245 dps
        self.i2c.write(LSM6DS33_ADDR, &[CTRL2_G, 0x80]).map_err(|_| ())?;

        Ok(())
    }

    pub fn read_accel(&mut self) -> Result<[i16; 3], ()> {
        let mut buf = [0u8; 6];
        self.i2c.write_read(LSM6DS33_ADDR, &[OUTX_L_XL], &mut buf).map_err(|_| ())?;
        
        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);

        Ok([x, y, z])
    }

    pub fn read_gyro(&mut self) -> Result<[i16; 3], ()> {
        let mut buf = [0u8; 6];
        self.i2c.write_read(LSM6DS33_ADDR, &[OUTX_L_G], &mut buf).map_err(|_| ())?;
        
        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);

        Ok([x, y, z])
    }
}