use num::{Num, Float, Signed};
use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign, Div, DivAssign, Neg};
use super::Quat;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vec4<T> {
    pub elements: [T; 4]
}

impl<T> Vec4<T> {
    pub const fn new(x: T, y: T, z: T, w: T) -> Self {
        Self {
            elements: [x, y, z, w]
        }
    }
}

impl<T> From<[T; 4]> for Vec4<T> {
    fn from(elements: [T; 4]) -> Self {
        Self {
            elements
        }
    }
}

impl From<Vec4<f32>> for Vec4<f64> {
    fn from(v: Vec4<f32>) -> Self {
        Self::new(v.x() as f64, v.y() as f64, v.z() as f64, v.w() as f64)
    }
}

impl From<Vec4<f64>> for Vec4<f32> {
    fn from(v: Vec4<f64>) -> Self {
        Self::new(v.x() as f32, v.y() as f32, v.z() as f32, v.w() as f32)
    }
}

impl Vec4<f32> {
    pub fn as_bytes(&self) -> [u8; 16] {
        let mut bytes = [0u8; 16];

        for (i, v) in self.elements.iter().enumerate() {
            let offset = i * std::mem::size_of::<f32>();

            let b = v.to_ne_bytes();

            bytes[offset + 0] = b[0];
            bytes[offset + 1] = b[1];
            bytes[offset + 2] = b[2];
            bytes[offset + 3] = b[3];
        }

        bytes
    }

    /// Creates a Vec4<f32> from a slice of 16 bytes.
    pub fn from_bytes(bytes: &[u8; 16]) -> Self {
        let mut elements = [0.0f32; 4];
        for i in 0..4 {

            let offset = i * 4;
            elements[i] = f32::from_ne_bytes([bytes[offset + 0], bytes[offset + 1], bytes[offset + 2], bytes[offset + 3]]);

        }

        Self {
            elements
        }
    }
}

impl Vec4<f64> {
    pub fn as_bytes(&self) -> [u8; 32] {
        let mut bytes = [0u8; 32];

        for (i, v) in self.elements.iter().enumerate() {
            let offset = i * std::mem::size_of::<f64>();
            let b = v.to_ne_bytes();

            bytes[offset + 0] = b[0];
            bytes[offset + 1] = b[1];
            bytes[offset + 2] = b[2];
            bytes[offset + 3] = b[3];
            bytes[offset + 4] = b[4];
            bytes[offset + 5] = b[5];
            bytes[offset + 6] = b[6];
            bytes[offset + 7] = b[7];
        }

        bytes
    }

    /// Creates a Vec4<f64> from a slice of 32 bytes.
    pub fn from_bytes(bytes: &[u8; 32]) -> Self {
        let mut elements = [0.0f64; 4];
        for i in 0..4 {

            let offset = i * 8;
            elements[i] = f64::from_ne_bytes([
                bytes[offset + 0],
                bytes[offset + 1],
                bytes[offset + 2],
                bytes[offset + 3],
                bytes[offset + 4],
                bytes[offset + 5],
                bytes[offset + 6],
                bytes[offset + 7]
            ]);

        }

        Self {
            elements
        }
    }
}

impl<T: Num + Copy> Vec4<T> {
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

    pub fn dot(&self, other: &Self) -> T {
        self.elements[0] * other.elements[0] + self.elements[1] * other.elements[1] + self.elements[2] * other.elements[2] + self.elements[3] * other.elements[3]
    }
}

impl<T: Num + Copy + Float> Vec4<T> {
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

    pub fn normalize(&mut self) {
        let mut ls = self.length_squared();
        if ls > T::zero() {
            ls = T::one() / ls.sqrt();
        }

        self.elements[0] = self.elements[0] * ls;
        self.elements[1] = self.elements[1] * ls;
        self.elements[2] = self.elements[2] * ls;
        self.elements[3] = self.elements[3] * ls;
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

impl<T, Index> std::ops::Index<Index> for Vec4<T>
where
    Index: std::slice::SliceIndex<[T]>
{
    type Output = Index::Output;

    fn index(&self, index: Index) -> &Self::Output {
        &self.elements[index]
    }
}

impl<T, Index> std::ops::IndexMut<Index> for Vec4<T>
where
    Index: std::slice::SliceIndex<[T]>
{
    fn index_mut(&mut self, index: Index) -> &mut Self::Output {
        &mut self.elements[index]
    }
}

impl<T: Num + Copy> Add for Vec4<T> {
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

impl<T: Num + Copy + AddAssign> AddAssign for Vec4<T> {
    fn add_assign(&mut self, other: Self) {
        self.elements[0] += other.elements[0];
        self.elements[1] += other.elements[1];
        self.elements[2] += other.elements[2];
        self.elements[3] += other.elements[3];
    }
}

impl<T: Num + Copy> Sub for Vec4<T> {
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

impl<T: Num + Copy + SubAssign> SubAssign for Vec4<T> {
    fn sub_assign(&mut self, other: Self) {
        self.elements[0] -= other.elements[0];
        self.elements[1] -= other.elements[1];
        self.elements[2] -= other.elements[2];
        self.elements[3] -= other.elements[3];
    }
}

impl<T: Num + Copy + Signed> Neg for Vec4<T> {
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

impl<T: Num + Copy> Mul for Vec4<T> {
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

impl<T: Num + Copy + MulAssign> MulAssign for Vec4<T> {
    fn mul_assign(&mut self, other: Self) {
        self.elements[0] *= other.elements[0];
        self.elements[1] *= other.elements[1];
        self.elements[2] *= other.elements[2];
        self.elements[3] *= other.elements[3];
    }
}

impl<T: Num + Copy> Mul<T> for Vec4<T> {
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

impl<T: Num + Copy + MulAssign> MulAssign<T> for Vec4<T> {
    fn mul_assign(&mut self, other: T) {
        self.elements[0] *= other;
        self.elements[1] *= other;
        self.elements[2] *= other;
        self.elements[3] *= other;
    }
}

impl<T: Num + Copy> Div for Vec4<T> {
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

impl<T: Num + Copy + DivAssign> DivAssign for Vec4<T> {
    fn div_assign(&mut self, other: Self) {
        self.elements[0] /= other.elements[0];
        self.elements[1] /= other.elements[1];
        self.elements[2] /= other.elements[2];
        self.elements[3] /= other.elements[3];
    }
}

impl<T: Num + Copy> Div<T> for Vec4<T> {
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

impl<T: Num + Copy + DivAssign> DivAssign<T> for Vec4<T> {
    fn div_assign(&mut self, other: T) {
        self.elements[0] /= other;
        self.elements[1] /= other;
        self.elements[2] /= other;
        self.elements[3] /= other;
    }
}