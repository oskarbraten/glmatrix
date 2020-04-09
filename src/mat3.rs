use num::{Num, Float};
use std::convert::From;
use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign};
use super::{Vec2, Mat4};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Mat3<T> {
    pub elements: [T; 9]
}

impl<T> Mat3<T> {
    pub const fn new(m00: T, m01: T, m02: T, m10: T, m11: T, m12: T, m20: T, m21: T, m22: T) -> Self {
        Self {
            elements: [m00, m01, m02, m10, m11, m12, m20, m21, m22]
        }
    }
}

impl<T> From<[T; 9]> for Mat3<T> {
    fn from(elements: [T; 9]) -> Self {
        Self {
            elements
        }
    }
}

impl<T: Num + Copy> Mat3<T> {
    /// Converts the matrix to a Mat4<T> by padding with zeros.
    /// Useful when the data layout of the target is defined as std140.
    pub fn as_mat4(self) -> Mat4<T> {
        Mat4::new(
            self.elements[0], self.elements[1], self.elements[2], T::zero(),
            self.elements[3], self.elements[4], self.elements[5], T::zero(), 
            self.elements[6], self.elements[7], self.elements[8], T::zero(),
            T::zero(), T::zero(), T::zero(), T::zero()
        )
    }
}

impl Mat3<f32> {
    pub fn as_bytes(&self) -> [u8; 36] {
        let mut bytes = [0u8; 36];

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

    /// Creates a Mat3<f32> from a slice of 36 bytes.
    pub fn from_bytes(bytes: &[u8; 36]) -> Self {
        let mut elements = [0.0f32; 9];
        for i in 0..9 {

            let offset = i * 4;
            elements[i] = f32::from_ne_bytes([bytes[offset + 0], bytes[offset + 1], bytes[offset + 2], bytes[offset + 3]]);

        }

        Self {
            elements
        }
    }
}

impl Mat3<f64> {
    pub fn as_bytes(&self) -> [u8; 72] {
        let mut bytes = [0u8; 72];

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

    /// Creates a Mat3<f64> from a slice of 72 bytes.
    pub fn from_bytes(bytes: &[u8; 72]) -> Self {
        let mut elements = [0.0f64; 9];
        for i in 0..9 {

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

impl<T: Float> Mat3<T> {
    pub fn identity() -> Self {
        let mut m = Self {
            elements: [T::zero(); 9]
        };
        
        m.elements[0] = T::one();
        m.elements[4] = T::one();
        m.elements[8] = T::one();
        
        m
    }
    
    pub fn transposed(&self) -> Self {
        Self::new(
            self.elements[0],
            self.elements[3],
            self.elements[6],
            self.elements[1],
            self.elements[4],
            self.elements[7],
            self.elements[2],
            self.elements[5],
            self.elements[8]
        )
    }
        
    pub fn determinant(&self) -> T {
        let [a00, a01, a02, a10, a11, a12, a20, a21, a22] = self.elements;
        a00 * (a22 * a11 - a12 * a21) + a01 * (-a22 * a10 + a12 * a20) + a02 * (a21 * a10 - a11 * a20)
    }
    
    pub fn inverse(&self) -> Self {
        let [a00, a01, a02, a10, a11, a12, a20, a21, a22] = self.elements;

        let b01 = a22 * a11 - a12 * a21;
        let b11 = -a22 * a10 + a12 * a20;
        let b21 = a21 * a10 - a11 * a20;

        // Calculate the determinant
        let inverse_det = T::one() / (a00 * b01 + a01 * b11 + a02 * b21);

        Self::new(
            b01 * inverse_det,
            (-a22 * a01 + a02 * a21) * inverse_det,
            (a12 * a01 - a02 * a11) * inverse_det,
            b11 * inverse_det,
            (a22 * a00 - a02 * a20) * inverse_det,
            (-a12 * a00 + a02 * a10) * inverse_det,
            b21 * inverse_det,
            (-a21 * a00 + a01 * a20) * inverse_det,
            (a11 * a00 - a01 * a10) * inverse_det
        )
    }

    pub fn adjoint(&self) -> Self {
        let [a00, a01, a02, a10, a11, a12, a20, a21, a22] = self.elements;

        Self::new(
        a11 * a22 - a12 * a21,
        a02 * a21 - a01 * a22,
        a01 * a12 - a02 * a11,
        a12 * a20 - a10 * a22,
        a00 * a22 - a02 * a20,
        a02 * a10 - a00 * a12,
        a10 * a21 - a11 * a20,
        a01 * a20 - a00 * a21,
        a00 * a11 - a01 * a10
        )
    }

    pub fn translate(&mut self, v: Vec2<T>) {
        let [a00, a01, a02, a10, a11, a12, a20, a21, a22] = self.elements;
        let [x, y] = v.elements;

        self.elements[0] = a00;
        self.elements[1] = a01;
        self.elements[2] = a02;
        self.elements[3] = a10;
        self.elements[4] = a11;
        self.elements[5] = a12;
        self.elements[6] = x * a00 + y * a10 + a20;
        self.elements[7] = x * a01 + y * a11 + a21;
        self.elements[8] = x * a02 + y * a12 + a22;
    }

    pub fn translated(self, v: Vec2<T>) -> Self {
        let [a00, a01, a02, a10, a11, a12, a20, a21, a22] = self.elements;
        let [x, y] = v.elements;

        Self::new(
            a00,
            a01,
            a02,
            a10,
            a11,
            a12,
            x * a00 + y * a10 + a20,
            x * a01 + y * a11 + a21,
            x * a02 + y * a12 + a22
        )
    }

    pub fn rotate(&mut self, rad: T) {
        let [a00, a01, a02, a10, a11, a12, a20, a21, a22] = self.elements;
        let s = rad.sin();
        let c = rad.cos();

        self.elements[0] = c * a00 + s * a10;
        self.elements[1] = c * a01 + s * a11;
        self.elements[2] = c * a02 + s * a12;
        self.elements[3] = c * a10 - s * a00;
        self.elements[4] = c * a11 - s * a01;
        self.elements[5] = c * a12 - s * a02;
        self.elements[6] = a20;
        self.elements[7] = a21;
        self.elements[8] = a22;
    }

    pub fn rotated(self, rad: T) -> Self {
        let [a00, a01, a02, a10, a11, a12, a20, a21, a22] = self.elements;
        let s = rad.sin();
        let c = rad.cos();

        Self::new(
            c * a00 + s * a10,
            c * a01 + s * a11,
            c * a02 + s * a12,
            c * a10 - s * a00,
            c * a11 - s * a01,
            c * a12 - s * a02,
            a20,
            a21,
            a22
        )
    }

    pub fn scale(&mut self, v: Vec2<T>) {
        let [a00, a01, a02, a10, a11, a12, a20, a21, a22] = self.elements;
        let [x, y] = v.elements;

        self.elements[0] = x * a00;
        self.elements[1] = x * a01;
        self.elements[2] = x * a02;
        self.elements[3] = y * a10;
        self.elements[4] = y * a11;
        self.elements[5] = y * a12;
        self.elements[6] = a20;
        self.elements[7] = a21;
        self.elements[8] = a22;
    }

    pub fn scaled(self, v: Vec2<T>) -> Self {
        let [a00, a01, a02, a10, a11, a12, a20, a21, a22] = self.elements;
        let [x, y] = v.elements;

        Self::new(
            x * a00,
            x * a01,
            x * a02,
            y * a10,
            y * a11,
            y * a12,
            a20,
            a21,
            a22
        )
    }

    pub fn from_translation(t: Vec2<T>) -> Self {
        Self::new(
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            t.elements[0],
            t.elements[1],
            T::one()
        )
    }

    pub fn from_rotation(rad: T) -> Self {
        let s = rad.sin();
        let c = rad.cos();
    
        Self::new(
            c,
            s,
            T::zero(),
            -s,
            c,
            T::zero(),
            T::zero(),
            T::zero(),
            T::one()
        )
    }

    pub fn from_scaling(s: Vec2<T>) -> Self {
        Self::new(
            s.elements[0],
            T::zero(),
            T::zero(),
            T::zero(),
            s.elements[1],
            T::zero(),
            T::zero(),
            T::zero(),
            T::one()
        )
    }
}

impl<T, Index> std::ops::Index<Index> for Mat3<T>
where
    Index: std::slice::SliceIndex<[T]>
{
    type Output = Index::Output;

    fn index(&self, index: Index) -> &Self::Output {
        &self.elements[index]
    }
}

impl<T, Index> std::ops::IndexMut<Index> for Mat3<T>
where
    Index: std::slice::SliceIndex<[T]>
{
    fn index_mut(&mut self, index: Index) -> &mut Self::Output {
        &mut self.elements[index]
    }
}

impl<T: Float> Add for Mat3<T> {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self::new(
            self.elements[0] + other.elements[0],
            self.elements[1] + other.elements[1],
            self.elements[2] + other.elements[2],
            self.elements[3] + other.elements[3],
            self.elements[4] + other.elements[4],
            self.elements[5] + other.elements[5],
            self.elements[6] + other.elements[6],
            self.elements[7] + other.elements[7],
            self.elements[8] + other.elements[8]
        )
    }
}

impl<T: Float + AddAssign> AddAssign for Mat3<T> {
    fn add_assign(&mut self, other: Self) {
        self.elements[0] += other.elements[0];
        self.elements[1] += other.elements[1];
        self.elements[2] += other.elements[2];
        self.elements[3] += other.elements[3];
        self.elements[4] += other.elements[4];
        self.elements[5] += other.elements[5];
        self.elements[6] += other.elements[6];
        self.elements[7] += other.elements[7];
        self.elements[8] += other.elements[8];
    }
}

impl<T: Float> Sub for Mat3<T> {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self::new(
            self.elements[0] - other.elements[0],
            self.elements[1] - other.elements[1],
            self.elements[2] - other.elements[2],
            self.elements[3] - other.elements[3],
            self.elements[4] - other.elements[4],
            self.elements[5] - other.elements[5],
            self.elements[6] - other.elements[6],
            self.elements[7] - other.elements[7],
            self.elements[8] - other.elements[8]
        )
    }
}

impl<T: Float + SubAssign> SubAssign for Mat3<T> {
    fn sub_assign(&mut self, other: Self) {
        self.elements[0] -= other.elements[0];
        self.elements[1] -= other.elements[1];
        self.elements[2] -= other.elements[2];
        self.elements[3] -= other.elements[3];
        self.elements[4] -= other.elements[4];
        self.elements[5] -= other.elements[5];
        self.elements[6] -= other.elements[6];
        self.elements[7] -= other.elements[7];
        self.elements[8] -= other.elements[8];
    }
}

impl<T: Float> Mul for Mat3<T> {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        let [a00, a01, a02, a10, a11, a12, a20, a21, a22] = self.elements;
        let [b00, b01, b02, b10, b11, b12, b20, b21, b22] = other.elements;


        Self::new(
            b00 * a00 + b01 * a10 + b02 * a20,
            b00 * a01 + b01 * a11 + b02 * a21,
            b00 * a02 + b01 * a12 + b02 * a22,
            b10 * a00 + b11 * a10 + b12 * a20,
            b10 * a01 + b11 * a11 + b12 * a21,
            b10 * a02 + b11 * a12 + b12 * a22,
            b20 * a00 + b21 * a10 + b22 * a20,
            b20 * a01 + b21 * a11 + b22 * a21,
            b20 * a02 + b21 * a12 + b22 * a22
        )
    }
}

impl<T: Float> MulAssign for Mat3<T> {
    fn mul_assign(&mut self, other: Self) {
        let [a00, a01, a02, a10, a11, a12, a20, a21, a22] = self.elements;
        let [b00, b01, b02, b10, b11, b12, b20, b21, b22] = other.elements;

        self.elements[0] = b00 * a00 + b01 * a10 + b02 * a20;
        self.elements[1] = b00 * a01 + b01 * a11 + b02 * a21;
        self.elements[2] = b00 * a02 + b01 * a12 + b02 * a22;
        self.elements[3] = b10 * a00 + b11 * a10 + b12 * a20;
        self.elements[4] = b10 * a01 + b11 * a11 + b12 * a21;
        self.elements[5] = b10 * a02 + b11 * a12 + b12 * a22;
        self.elements[6] = b20 * a00 + b21 * a10 + b22 * a20;
        self.elements[7] = b20 * a01 + b21 * a11 + b22 * a21;
        self.elements[8] = b20 * a02 + b21 * a12 + b22 * a22;
    }
}

impl<T: Float> Mul<T> for Mat3<T> {
    type Output = Self;
    
    fn mul(self, other: T) -> Self {
        Self::new(
            self.elements[0] * other,
            self.elements[1] * other,
            self.elements[2] * other,
            self.elements[3] * other,
            self.elements[4] * other,
            self.elements[5] * other,
            self.elements[6] * other,
            self.elements[7] * other,
            self.elements[8] * other
        )
    }
}

impl<T: Float + MulAssign> MulAssign<T> for Mat3<T> {
    fn mul_assign(&mut self, other: T) {
        self.elements[0] *= other;
        self.elements[1] *= other;
        self.elements[2] *= other;
        self.elements[3] *= other;
        self.elements[4] *= other;
        self.elements[5] *= other;
        self.elements[6] *= other;
        self.elements[7] *= other;
        self.elements[8] *= other;
    }
}