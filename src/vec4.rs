use num::Float;
use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign, Div, DivAssign, Neg};
use super::Quat;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vec4<T: Float> {
    pub elements: [T; 4]
}

impl<T: Float + MulAssign> Vec4<T> {
    pub fn new(x: T, y: T, z: T, w: T) -> Self {
        Self {
            elements: [x, y, z, w]
        }
    }

    pub fn zero() -> Self {
        Self {
            elements: [T::zero(); 4]
        }
    }

    pub fn one() -> Self {
        Self {
            elements: [T::one(); 4]
        }
    }

    pub fn x(&self) -> T {
        self.elements[0]
    }

    pub fn y(&self) -> T {
        self.elements[1]
    }

    pub fn z(&self) -> T {
        self.elements[2]
    }

    pub fn w(&self) -> T {
        self.elements[3]
    }

    pub fn set_x(&mut self, value: T) {
        self.elements[0] = value;
    }

    pub fn set_y(&mut self, value: T) {
        self.elements[1] = value;
    }

    pub fn set_z(&mut self, value: T) {
        self.elements[2] = value;
    }

    pub fn set_w(&mut self, value: T) {
        self.elements[3] = value;
    }

    pub fn length_squared(&self) -> T {
        self.elements[0].powi(2) + self.elements[1].powi(2) + self.elements[2].powi(2) + self.elements[3].powi(2)
    }

    pub fn length(&self) -> T {
        self.length_squared().sqrt()
    }

    pub fn magnitude(&self) -> T {
        self.length()
    }

    pub fn dist_squared(&self, other: &Self) -> T {
        let [ax, ay, az, aw] = self.elements;
        let [bx, by, bz, bw] = other.elements;

        (bx - ax).powi(2) + (by - ay).powi(2) + (bz - az).powi(2) + (bw - aw).powi(2)
    }

    pub fn dist(&self, other: &Self) -> T {
        self.dist_squared(other).sqrt()
    }

    pub fn invert(&mut self) {
        self.elements[0] = T::one() / self.elements[0];
        self.elements[1] = T::one() / self.elements[1];
        self.elements[2] = T::one() / self.elements[2];
        self.elements[3] = T::one() / self.elements[3];
    }

    pub fn inverse(&self) -> Self {
        Self::new(
            T::one() / self.elements[0],
            T::one() / self.elements[1],
            T::one() / self.elements[2],
            T::one() / self.elements[3]
        )
    }

    pub fn normalize(&mut self) {
        let mut ls = self.length_squared();
        if ls > T::zero() {
            ls = T::one() / ls.sqrt();
        }

        self.elements[0] *= ls;
        self.elements[1] *= ls;
        self.elements[2] *= ls;
        self.elements[3] *= ls;
    }

    pub fn normalized(&self) -> Self {
        let mut ls = self.length_squared();
        if ls > T::zero() {
            ls = T::one() / ls.sqrt();
        }

        Self::new(
            self.elements[0] * ls,
            self.elements[1] * ls,
            self.elements[2] * ls,
            self.elements[3] * ls
        )
    }

    pub fn dot(&self, other: &Self) -> T {
        self.elements[0] * other.elements[0] + self.elements[1] * other.elements[1] + self.elements[2] * other.elements[2] + self.elements[3] * other.elements[3]
    }

    pub fn lerp(&self, other: &Self, t: T) -> Self {
        Self::new(
            self.elements[0] + t * (other.elements[0] - self.elements[0]),
            self.elements[1] + t * (other.elements[1] - self.elements[1]),
            self.elements[2] + t * (other.elements[2] - self.elements[2]),
            self.elements[3] + t * (other.elements[3] - self.elements[3])
        )
    }

    pub fn rotate_quat(&mut self, quat: Quat<T>) {
        let [x, y, z, _] = self.elements;
        let [qx, qy, qz, qw] = quat.elements;

        let ix = qw * x + qy * z - qz * y;
        let iy = qw * y + qz * x - qx * z;
        let iz = qw * z + qx * y - qy * x;
        let iw = -qx * x - qy * y - qz * z;

        self.elements[0] = ix * qw + iw * -qx + iy * -qz - iz * -qy;
        self.elements[1] = iy * qw + iw * -qy + iz * -qx - ix * -qz;
        self.elements[2] = iz * qw + iw * -qz + ix * -qy - iy * -qx;
    }
}

impl<T: Float + MulAssign> Add for Vec4<T> {
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

impl<T: Float + MulAssign + AddAssign> AddAssign for Vec4<T> {
    fn add_assign(&mut self, other: Self) {
        self.elements[0] += other.elements[0];
        self.elements[1] += other.elements[1];
        self.elements[2] += other.elements[2];
        self.elements[3] += other.elements[3];
    }
}

impl<T: Float + MulAssign> Sub for Vec4<T> {
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

impl<T: Float + MulAssign + SubAssign> SubAssign for Vec4<T> {
    fn sub_assign(&mut self, other: Self) {
        self.elements[0] -= other.elements[0];
        self.elements[1] -= other.elements[1];
        self.elements[2] -= other.elements[2];
        self.elements[3] -= other.elements[3];
    }
}

impl<T: Float + MulAssign> Neg for Vec4<T> {
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

impl<T: Float + MulAssign> Mul for Vec4<T> {
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

impl<T: Float + MulAssign> MulAssign for Vec4<T> {
    fn mul_assign(&mut self, other: Self) {
        self.elements[0] *= other.elements[0];
        self.elements[1] *= other.elements[1];
        self.elements[2] *= other.elements[2];
        self.elements[3] *= other.elements[3];
    }
}

impl<T: Float + MulAssign> Mul<T> for Vec4<T> {
    type Output = Self;

    fn mul(self, other: T) -> Self {
        Self::new(
            self.elements[0] * other,
            self.elements[1] * other,
            self.elements[2] * other,
            self.elements[3] * other
        )
    }
}

impl<T: Float + MulAssign> MulAssign<T> for Vec4<T> {
    fn mul_assign(&mut self, other: T) {
        self.elements[0] *= other;
        self.elements[1] *= other;
        self.elements[2] *= other;
        self.elements[3] *= other;
    }
}

impl<T: Float + MulAssign> Div for Vec4<T> {
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

impl<T: Float + MulAssign + DivAssign> DivAssign for Vec4<T> {
    fn div_assign(&mut self, other: Self) {
        self.elements[0] /= other.elements[0];
        self.elements[1] /= other.elements[1];
        self.elements[2] /= other.elements[2];
        self.elements[3] /= other.elements[3];
    }
}

impl<T: Float + MulAssign> Div<T> for Vec4<T> {
    type Output = Vec4<T>;

    fn div(self, other: T) -> Self {
        Self::new(
            self.elements[0] / other,
            self.elements[1] / other,
            self.elements[2] / other,
            self.elements[3] / other
        )
    }
}

impl<T: Float + MulAssign + DivAssign> DivAssign<T> for Vec4<T> {
    fn div_assign(&mut self, other: T) {
        self.elements[0] /= other;
        self.elements[1] /= other;
        self.elements[2] /= other;
        self.elements[3] /= other;
    }
}