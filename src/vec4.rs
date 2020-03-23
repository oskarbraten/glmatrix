use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign, Div, DivAssign, Neg};
use super::Quat;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vec4 {
    pub elements: [f32; 4]
}

impl Vec4 {
    pub const fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self {
            elements: [x, y, z, w]
        }
    }

    pub const fn zero() -> Self {
        Self {
            elements: [0.0; 4]
        }
    }

    pub const fn one() -> Self {
        Self {
            elements: [1.0; 4]
        }
    }

    pub fn x(&self) -> f32 {
        self.elements[0]
    }

    pub fn y(&self) -> f32 {
        self.elements[1]
    }

    pub fn z(&self) -> f32 {
        self.elements[2]
    }

    pub fn w(&self) -> f32 {
        self.elements[3]
    }

    pub fn set_x(&mut self, value: f32) {
        self.elements[0] = value;
    }

    pub fn set_y(&mut self, value: f32) {
        self.elements[1] = value;
    }

    pub fn set_z(&mut self, value: f32) {
        self.elements[2] = value;
    }

    pub fn set_w(&mut self, value: f32) {
        self.elements[3] = value;
    }

    pub fn length_squared(&self) -> f32 {
        self.elements[0].powi(2) + self.elements[1].powi(2) + self.elements[2].powi(2) + self.elements[3].powi(2)
    }

    pub fn length(&self) -> f32 {
        self.length_squared().sqrt()
    }

    pub fn magnitude(&self) -> f32 {
        self.length()
    }

    pub fn dist_squared(&self, other: &Self) -> f32 {
        let [ax, ay, az, aw] = &self.elements;
        let [bx, by, bz, bw] = &other.elements;

        (bx - ax).powi(2) + (by - ay).powi(2) + (bz - az).powi(2) + (bw - aw).powi(2)
    }

    pub fn dist(&self, other: &Self) -> f32 {
        self.dist_squared(other).sqrt()
    }

    pub fn invert(&mut self) {
        self.elements[0] = 1.0 / self.elements[0];
        self.elements[1] = 1.0 / self.elements[1];
        self.elements[2] = 1.0 / self.elements[2];
        self.elements[3] = 1.0 / self.elements[3];
    }

    pub fn inverse(&self) -> Self {
        Self::new(
            1.0 / self.elements[0],
            1.0 / self.elements[1],
            1.0 / self.elements[2],
            1.0 / self.elements[3]
        )
    }

    pub fn normalize(&mut self) {
        let mut ls = self.length_squared();
        if ls > 0.0 {
            ls = 1.0 / ls.sqrt();
        }

        self.elements[0] *= ls;
        self.elements[1] *= ls;
        self.elements[2] *= ls;
        self.elements[3] *= ls;
    }

    pub fn normalized(&self) -> Self {
        let mut ls = self.length_squared();
        if ls > 0.0 {
            ls = 1.0 / ls.sqrt();
        }

        Self::new(
            self.elements[0] * ls,
            self.elements[1] * ls,
            self.elements[2] * ls,
            self.elements[3] * ls
        )
    }

    pub fn dot(&self, other: &Self) -> f32 {
        self.elements[0] * other.elements[0] + self.elements[1] * other.elements[1] + self.elements[2] * other.elements[2] + self.elements[3] * other.elements[3]
    }

    pub fn lerp(&self, other: &Self, t: f32) -> Self {
        Self::new(
            self.elements[0] + t * (other.elements[0] - self.elements[0]),
            self.elements[1] + t * (other.elements[1] - self.elements[1]),
            self.elements[2] + t * (other.elements[2] - self.elements[2]),
            self.elements[3] + t * (other.elements[3] - self.elements[3])
        )
    }

    pub fn rotate_quat(&mut self, quat: Quat) {
        let [x, y, z, _] = &self.elements;
        let [qx, qy, qz, qw] = &quat.elements;

        let ix = qw * x + qy * z - qz * y;
        let iy = qw * y + qz * x - qx * z;
        let iz = qw * z + qx * y - qy * x;
        let iw = -qx * x - qy * y - qz * z;

        self.elements[0] = ix * qw + iw * -qx + iy * -qz - iz * -qy;
        self.elements[1] = iy * qw + iw * -qy + iz * -qx - ix * -qz;
        self.elements[2] = iz * qw + iw * -qz + ix * -qy - iy * -qx;
    }
}

impl Add for Vec4 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self::new(
            self.elements[0] + other.elements[0],
            self.elements[1] + other.elements[1],
            self.elements[2] + other.elements[2],
            self.elements[3] + other.elements[3]
        )
    }
}

impl AddAssign for Vec4 {
    fn add_assign(&mut self, other: Self) {
        self.elements[0] += other.elements[0];
        self.elements[1] += other.elements[1];
        self.elements[2] += other.elements[2];
        self.elements[3] += other.elements[3];
    }
}

impl Sub for Vec4 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self::new(
            self.elements[0] - other.elements[0],
            self.elements[1] - other.elements[1],
            self.elements[2] - other.elements[2],
            self.elements[3] - other.elements[3]
        )
    }
}

impl SubAssign for Vec4 {
    fn sub_assign(&mut self, other: Self) {
        self.elements[0] -= other.elements[0];
        self.elements[1] -= other.elements[1];
        self.elements[2] -= other.elements[2];
        self.elements[3] -= other.elements[3];
    }
}

impl Neg for Vec4 {
    type Output = Self;

    fn neg(self) -> Self {
        Self::new(
            -self.elements[0],
            -self.elements[1],
            -self.elements[2],
            -self.elements[3]
        )
    }
}

impl Mul for Vec4 {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        Self::new(
            self.elements[0] * other.elements[0],
            self.elements[1] * other.elements[1],
            self.elements[2] * other.elements[2],
            self.elements[3] * other.elements[3]
        )
    }
}

impl MulAssign for Vec4 {
    fn mul_assign(&mut self, other: Self) {
        self.elements[0] *= other.elements[0];
        self.elements[1] *= other.elements[1];
        self.elements[2] *= other.elements[2];
        self.elements[3] *= other.elements[3];
    }
}

impl Mul<f32> for Vec4 {
    type Output = Self;

    fn mul(self, other: f32) -> Self {
        Self::new(
            self.elements[0] * other,
            self.elements[1] * other,
            self.elements[2] * other,
            self.elements[3] * other
        )
    }
}

impl MulAssign<f32> for Vec4 {
    fn mul_assign(&mut self, other: f32) {
        self.elements[0] *= other;
        self.elements[1] *= other;
        self.elements[2] *= other;
        self.elements[3] *= other;
    }
}

impl Div for Vec4 {
    type Output = Self;

    fn div(self, other: Self) -> Self {
        Self::new(
            self.elements[0] / other.elements[0],
            self.elements[1] / other.elements[1],
            self.elements[2] / other.elements[2], 
            self.elements[3] / other.elements[3]
        )
    }
}

impl DivAssign for Vec4 {
    fn div_assign(&mut self, other: Self) {
        self.elements[0] /= other.elements[0];
        self.elements[1] /= other.elements[1];
        self.elements[2] /= other.elements[2];
        self.elements[3] /= other.elements[3];
    }
}

impl Div<f32> for Vec4 {
    type Output = Vec4;

    fn div(self, other: f32) -> Self {
        Self::new(
            self.elements[0] / other,
            self.elements[1] / other,
            self.elements[2] / other,
            self.elements[3] / other
        )
    }
}

impl DivAssign<f32> for Vec4 {
    fn div_assign(&mut self, other: f32) {
        self.elements[0] /= other;
        self.elements[1] /= other;
        self.elements[2] /= other;
        self.elements[3] /= other;
    }
}