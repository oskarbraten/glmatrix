use num::Float;
use std::convert::From;
use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign};
use super::{Vec3, Vec4, Quat, Mat3};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Mat4<T> {
    pub elements: [T; 16]
}

impl<T> Mat4<T> {
    pub const fn new(m00: T, m01: T, m02: T, m03: T, m10: T, m11: T, m12: T, m13: T, m20: T, m21: T, m22: T, m23: T, m30: T, m31: T, m32: T, m33: T) -> Self {
        Self {
            elements: [m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33]
        }
    }
}

impl Mat4<f32> {
    pub fn as_bytes(&self) -> [u8; 64] {
        let mut bytes = [0u8; 64];

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

    /// Creates a Mat4<f32> from a slice of 64 bytes.
    pub fn from_bytes(bytes: &[u8; 64]) -> Self {
        let mut elements = [0.0f32; 16];
        for i in 0..16 {

            let offset = i * 4;
            elements[i] = f32::from_ne_bytes([bytes[offset + 0], bytes[offset + 1], bytes[offset + 2], bytes[offset + 3]]);

        }

        Self {
            elements
        }
    }
}

impl Mat4<f64> {
    pub fn as_bytes(&self) -> [u8; 128] {
        let mut bytes = [0u8; 128];

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

    /// Creates a Mat4<f64> from a slice of 128 bytes.
    pub fn from_bytes(bytes: &[u8; 128]) -> Self {
        let mut elements = [0.0f64; 16];
        for i in 0..16 {

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

impl<T: Float> Mat4<T> {
    pub fn identity() -> Self {
        let mut m = Self {
            elements: [T::zero(); 16]
        };
        
        m.elements[0] = T::one();
        m.elements[5] = T::one();
        m.elements[10] = T::one();
        m.elements[15] = T::one();
        
        m
    }
    
    pub fn transposed(&self) -> Self {
        Self::new(
            self.elements[0],
            self.elements[4],
            self.elements[8],
            self.elements[12],
            self.elements[1],
            self.elements[5],
            self.elements[9],
            self.elements[13],
            self.elements[2],
            self.elements[6],
            self.elements[10],
            self.elements[14],
            self.elements[3],
            self.elements[7],
            self.elements[11],
            self.elements[15]
        )
    }
        
    pub fn determinant(&self) -> T {
        let [a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31, a32, a33] = self.elements;
        
        let b00 = a00 * a11 - a01 * a10;
        let b01 = a00 * a12 - a02 * a10;
        let b02 = a00 * a13 - a03 * a10;
        let b03 = a01 * a12 - a02 * a11;
        let b04 = a01 * a13 - a03 * a11;
        let b05 = a02 * a13 - a03 * a12;
        let b06 = a20 * a31 - a21 * a30;
        let b07 = a20 * a32 - a22 * a30;
        let b08 = a20 * a33 - a23 * a30;
        let b09 = a21 * a32 - a22 * a31;
        let b10 = a21 * a33 - a23 * a31;
        let b11 = a22 * a33 - a23 * a32;
        
        b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06
    }
    
    pub fn inverse(&self) -> Self {
        let [a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31, a32, a33] = self.elements;
        
        let b00 = a00 * a11 - a01 * a10;
        let b01 = a00 * a12 - a02 * a10;
        let b02 = a00 * a13 - a03 * a10;
        let b03 = a01 * a12 - a02 * a11;
        let b04 = a01 * a13 - a03 * a11;
        let b05 = a02 * a13 - a03 * a12;
        let b06 = a20 * a31 - a21 * a30;
        let b07 = a20 * a32 - a22 * a30;
        let b08 = a20 * a33 - a23 * a30;
        let b09 = a21 * a32 - a22 * a31;
        let b10 = a21 * a33 - a23 * a31;
        let b11 = a22 * a33 - a23 * a32;
        
        let inverse_det = T::one() / (b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06);
        
        Self::new(
            (a11 * b11 - a12 * b10 + a13 * b09) * inverse_det,
            (a02 * b10 - a01 * b11 - a03 * b09) * inverse_det,
            (a31 * b05 - a32 * b04 + a33 * b03) * inverse_det,
            (a22 * b04 - a21 * b05 - a23 * b03) * inverse_det,
            (a12 * b08 - a10 * b11 - a13 * b07) * inverse_det,
            (a00 * b11 - a02 * b08 + a03 * b07) * inverse_det,
            (a32 * b02 - a30 * b05 - a33 * b01) * inverse_det,
            (a20 * b05 - a22 * b02 + a23 * b01) * inverse_det,
            (a10 * b10 - a11 * b08 + a13 * b06) * inverse_det,
            (a01 * b08 - a00 * b10 - a03 * b06) * inverse_det,
            (a30 * b04 - a31 * b02 + a33 * b00) * inverse_det,
            (a21 * b02 - a20 * b04 - a23 * b00) * inverse_det,
            (a11 * b07 - a10 * b09 - a12 * b06) * inverse_det,
            (a00 * b09 - a01 * b07 + a02 * b06) * inverse_det,
            (a31 * b01 - a30 * b03 - a32 * b00) * inverse_det,
            (a20 * b03 - a21 * b01 + a22 * b00) * inverse_det
        )
    }

    pub fn adjoint(&self) -> Self {
        let [a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31, a32, a33] = self.elements;
    
        Self::new(
            a11 * (a22 * a33 - a23 * a32) - a21 * (a12 * a33 - a13 * a32) + a31 * (a12 * a23 - a13 * a22),
            -(a01 * (a22 * a33 - a23 * a32) - a21 * (a02 * a33 - a03 * a32) + a31 * (a02 * a23 - a03 * a22)),
            a01 * (a12 * a33 - a13 * a32) - a11 * (a02 * a33 - a03 * a32) + a31 * (a02 * a13 - a03 * a12),
            -(a01 * (a12 * a23 - a13 * a22) - a11 * (a02 * a23 - a03 * a22) + a21 * (a02 * a13 - a03 * a12)),
            -(a10 * (a22 * a33 - a23 * a32) - a20 * (a12 * a33 - a13 * a32) + a30 * (a12 * a23 - a13 * a22)),
            a00 * (a22 * a33 - a23 * a32) - a20 * (a02 * a33 - a03 * a32) + a30 * (a02 * a23 - a03 * a22),
            -(a00 * (a12 * a33 - a13 * a32) - a10 * (a02 * a33 - a03 * a32) + a30 * (a02 * a13 - a03 * a12)),
            a00 * (a12 * a23 - a13 * a22) - a10 * (a02 * a23 - a03 * a22) + a20 * (a02 * a13 - a03 * a12),
            a10 * (a21 * a33 - a23 * a31) - a20 * (a11 * a33 - a13 * a31) + a30 * (a11 * a23 - a13 * a21),
            -(a00 * (a21 * a33 - a23 * a31) - a20 * (a01 * a33 - a03 * a31) + a30 * (a01 * a23 - a03 * a21)),
            a00 * (a11 * a33 - a13 * a31) - a10 * (a01 * a33 - a03 * a31) + a30 * (a01 * a13 - a03 * a11),
            -(a00 * (a11 * a23 - a13 * a21) - a10 * (a01 * a23 - a03 * a21) + a20 * (a01 * a13 - a03 * a11)),
            -(a10 * (a21 * a32 - a22 * a31) - a20 * (a11 * a32 - a12 * a31) + a30 * (a11 * a22 - a12 * a21)),
            a00 * (a21 * a32 - a22 * a31) - a20 * (a01 * a32 - a02 * a31) + a30 * (a01 * a22 - a02 * a21),
            -(a00 * (a11 * a32 - a12 * a31) - a10 * (a01 * a32 - a02 * a31) + a30 * (a01 * a12 - a02 * a11)),
            a00 * (a11 * a22 - a12 * a21) - a10 * (a01 * a22 - a02 * a21) + a20 * (a01 * a12 - a02 * a11)
        )
    }

    pub fn translate(&mut self, v: Vec3<T>) {
        let [x, y, z] = v.elements;

        self.elements[12] = self.elements[0] * x + self.elements[4] * y + self.elements[8] * z + self.elements[12];
        self.elements[13] = self.elements[1] * x + self.elements[5] * y + self.elements[9] * z + self.elements[13];
        self.elements[14] = self.elements[2] * x + self.elements[6] * y + self.elements[10] * z + self.elements[14];
        self.elements[15] = self.elements[3] * x + self.elements[7] * y + self.elements[11] * z + self.elements[15];
    }

    pub fn translated(self, v: Vec3<T>) -> Self {
        let [x, y, z] = v.elements;
        let [a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31, a32, a33] = self.elements;
        
        Self::new(
            a00,
            a01,
            a02,
            a03,
            a10,
            a11,
            a12,
            a13,
            a20,
            a21,
            a22,
            a23,
            a00 * x + a10 * y + a20 * z + a30,
            a01 * x + a11 * y + a21 * z + a31,
            a02 * x + a12 * y + a22 * z + a32,
            a03 * x + a13 * y + a23 * z + a33
        )
    }

    pub fn scale(&mut self, v: Vec3<T>) {
        let [x, y, z] = v.elements;

        self.elements[0] = self.elements[0] * x;
        self.elements[1] = self.elements[1] * x;
        self.elements[2] = self.elements[2] * x;
        self.elements[3] = self.elements[3] * x;
        self.elements[4] = self.elements[4] * y;
        self.elements[5] = self.elements[5] * y;
        self.elements[6] = self.elements[6] * y;
        self.elements[7] = self.elements[7] * y;
        self.elements[8] = self.elements[8] * z;
        self.elements[9] = self.elements[9] * z;
        self.elements[10] = self.elements[10] * z;
        self.elements[11] = self.elements[11] * z;
    }

    pub fn scaled(self, v: Vec3<T>) -> Self {
        let [x, y, z] = v.elements;

        Self::new(
            self.elements[0] * x,
            self.elements[1] * x,
            self.elements[2] * x,
            self.elements[3] * x,
            self.elements[4] * y,
            self.elements[5] * y,
            self.elements[6] * y,
            self.elements[7] * y,
            self.elements[8] * z,
            self.elements[9] * z,
            self.elements[10] * z,
            self.elements[11] * z,
            self.elements[12],
            self.elements[13],
            self.elements[14],
            self.elements[15]
        )
    }

    pub fn get_translation(&self) -> Vec3<T> {
        Vec3::new(self.elements[12], self.elements[13], self.elements[14])
    }

    pub fn get_scaling(&self) -> Vec3<T> {
        Vec3::new(
            (self.elements[0].powi(2) + self.elements[2].powi(2) + self.elements[3].powi(2)).sqrt(),
            (self.elements[4].powi(2) + self.elements[5].powi(2) + self.elements[6].powi(2)).sqrt(),
            (self.elements[8].powi(2) + self.elements[9].powi(2) + self.elements[10].powi(2)).sqrt(),
        )
    }

    // pub fn get_rotation(&self) -> Quat<T> {
    //     unimplemented!()
    // }

    pub fn from_rotation(r: Quat<T>) -> Self {
        let [x, y, z, w] = r.elements;
    
        let x2 = x + x;
        let y2 = y + y;
        let z2 = z + z;
        let xx = x * x2;
        let yx = y * x2;
        let yy = y * y2;
        let zx = z * x2;
        let zy = z * y2;
        let zz = z * z2;
        let wx = w * x2;
        let wy = w * y2;
        let wz = w * z2;
    
        Self::new(
            T::one() - yy - zz,
            yx + wz,
            zx - wy,
            T::zero(),
            yx - wz,
            T::one() - xx - zz,
            zy + wx,
            T::zero(),
            zx + wy,
            zy - wx,
            T::one() - xx - yy,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one()
        )
    }

    pub fn from_translation(t: Vec3<T>) -> Self {
        Self::new(
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            t.x(),
            t.y(),
            t.z(),
            T::one()
        )
    }

    pub fn from_scaling(s: Vec3<T>) -> Self {
        Self::new(
            s.x(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            s.y(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            s.z(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one()
        )
    }

    pub fn from_translation_rotation_scale(t: Vec3<T>, r: Quat<T>, s: Vec3<T>) -> Self {
        let [x, y, z, w] = r.elements;
    
        let x2 = x + x;
        let y2 = y + y;
        let z2 = z + z;

        let xx = x * x2;
        let xy = x * y2;
        let xz = x * z2;
        let yy = y * y2;
        let yz = y * z2;
        let zz = z * z2;
        let wx = w * x2;
        let wy = w * y2;
        let wz = w * z2;

        let sx = s.x();
        let sy = s.y();
        let sz = s.z();
    
        Self::new(
            (T::one() - (yy + zz)) * sx,
            (xy + wz) * sx,
            (xz - wy) * sx,
            T::zero(),
            (xy - wz) * sy,
            (T::one() - (xx + zz)) * sy,
            (yz + wx) * sy,
            T::zero(),
            (xz + wy) * sz,
            (yz - wx) * sz,
            (T::one() - (xx + yy)) * sz,
            T::zero(),
            t.x(),
            t.y(),
            t.z(),
            T::one()
        )
    }

    pub fn perspective(fovy: T, aspect: T, near: T, far: T) -> Self {
        let f = T::one() / (fovy / (T::one() + T::one())).tan();
        let mut m = Self::new(
            f / aspect,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            f,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            -T::one(),
            -T::one(),
            T::zero(),
            T::zero(),
            -(T::one() + T::one()) * near,
            T::zero()
        );
    
        if far != T::infinity() {
            let nf = T::one() / (near - far);
            m.elements[10] = (far + near) * nf;
            m.elements[14] = (T::one() + T::one()) * far * near * nf;
        }

        m
    }

    pub fn ortho(left: T, right: T, bottom: T, top: T, near: T, far: T) -> Self {
        let lr = T::one() / (left - right);
        let bt = T::one() / (bottom - top);
        let nf = T::one() / (near - far);

        Self::new(
            -(T::one() + T::one()) * lr,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            -(T::one() + T::one()) * bt,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            (T::one() + T::one()) * nf,
            T::zero(),
            (left + right) * lr,
            (top + bottom) * bt,
            (far + near) * nf,
            T::one()
        )
    }

    /// Extracts the direction vector from the matrix.
    pub fn direction(&self) -> Vec3<T> {
        Vec3::new(self[8], self[9], self[10])
    }

    /// Calculates a 3x3 normal matrix (transpose inverse) from the 4x4 matrix
    pub fn normal_matrix(&self) -> Mat3<T> {
        let [a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31, a32, a33] = self.elements;

        let b00 = a00 * a11 - a01 * a10;
        let b01 = a00 * a12 - a02 * a10;
        let b02 = a00 * a13 - a03 * a10;
        let b03 = a01 * a12 - a02 * a11;
        let b04 = a01 * a13 - a03 * a11;
        let b05 = a02 * a13 - a03 * a12;
        let b06 = a20 * a31 - a21 * a30;
        let b07 = a20 * a32 - a22 * a30;
        let b08 = a20 * a33 - a23 * a30;
        let b09 = a21 * a32 - a22 * a31;
        let b10 = a21 * a33 - a23 * a31;
        let b11 = a22 * a33 - a23 * a32;

        let inverse_det = T::one() / b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;

        Mat3::new(
            (a11 * b11 - a12 * b10 + a13 * b09) * inverse_det,
            (a12 * b08 - a10 * b11 - a13 * b07) * inverse_det,
            (a10 * b10 - a11 * b08 + a13 * b06) * inverse_det,
            (a02 * b10 - a01 * b11 - a03 * b09) * inverse_det,
            (a00 * b11 - a02 * b08 + a03 * b07) * inverse_det,
            (a01 * b08 - a00 * b10 - a03 * b06) * inverse_det,
            (a31 * b05 - a32 * b04 + a33 * b03) * inverse_det,
            (a32 * b02 - a30 * b05 - a33 * b01) * inverse_det,
            (a30 * b04 - a31 * b02 + a33 * b00) * inverse_det
        )
    }
}

impl<T: Float> From<[T; 16]> for Mat4<T> {
    fn from(elements: [T; 16]) -> Self {
        Self {
            elements
        }
    }
}


impl<T, Index> std::ops::Index<Index> for Mat4<T>
where
    Index: std::slice::SliceIndex<[T]>
{
    type Output = Index::Output;

    fn index(&self, index: Index) -> &Self::Output {
        &self.elements[index]
    }
}

impl<T, Index> std::ops::IndexMut<Index> for Mat4<T>
where
    Index: std::slice::SliceIndex<[T]>
{
    fn index_mut(&mut self, index: Index) -> &mut Self::Output {
        &mut self.elements[index]
    }
}

impl<T: Float> Add for Mat4<T> {
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
            self.elements[8] + other.elements[8],
            self.elements[9] + other.elements[9],
            self.elements[10] + other.elements[10],
            self.elements[11] + other.elements[11],
            self.elements[12] + other.elements[12],
            self.elements[13] + other.elements[13],
            self.elements[14] + other.elements[14],
            self.elements[15] + other.elements[15]
        )
    }
}

impl<T: Float + AddAssign> AddAssign for Mat4<T> {
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
        self.elements[9] += other.elements[9];
        self.elements[10] += other.elements[10];
        self.elements[11] += other.elements[11];
        self.elements[12] += other.elements[12];
        self.elements[13] += other.elements[13];
        self.elements[14] += other.elements[14];
        self.elements[15] += other.elements[15];
    }
}

impl<T: Float> Sub for Mat4<T> {
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
            self.elements[8] - other.elements[8],
            self.elements[9] - other.elements[9],
            self.elements[10] - other.elements[10],
            self.elements[11] - other.elements[11],
            self.elements[12] - other.elements[12],
            self.elements[13] - other.elements[13],
            self.elements[14] - other.elements[14],
            self.elements[15] - other.elements[15]
        )
    }
}

impl<T: Float + SubAssign> SubAssign for Mat4<T> {
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
        self.elements[9] -= other.elements[9];
        self.elements[10] -= other.elements[10];
        self.elements[11] -= other.elements[11];
        self.elements[12] -= other.elements[12];
        self.elements[13] -= other.elements[13];
        self.elements[14] -= other.elements[14];
        self.elements[15] -= other.elements[15];
    }
}

impl<T: Float> Mul for Mat4<T> {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        let [a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31, a32, a33] = self.elements;
        let [b00, b01, b02, b03, b10, b11, b12, b13, b20, b21, b22, b23, b30, b31, b32, b33] = other.elements;

        Self::new(
            b00 * a00 + b01 * a10 + b02 * a20 + b03 * a30,
            b00 * a01 + b01 * a11 + b02 * a21 + b03 * a31,
            b00 * a02 + b01 * a12 + b02 * a22 + b03 * a32,
            b00 * a03 + b01 * a13 + b02 * a23 + b03 * a33,
            b10 * a00 + b11 * a10 + b12 * a20 + b13 * a30,
            b10 * a01 + b11 * a11 + b12 * a21 + b13 * a31,
            b10 * a02 + b11 * a12 + b12 * a22 + b13 * a32,
            b10 * a03 + b11 * a13 + b12 * a23 + b13 * a33,
            b20 * a00 + b21 * a10 + b22 * a20 + b23 * a30,
            b20 * a01 + b21 * a11 + b22 * a21 + b23 * a31,
            b20 * a02 + b21 * a12 + b22 * a22 + b23 * a32,
            b20 * a03 + b21 * a13 + b22 * a23 + b23 * a33,
            b30 * a00 + b31 * a10 + b32 * a20 + b33 * a30,
            b30 * a01 + b31 * a11 + b32 * a21 + b33 * a31,
            b30 * a02 + b31 * a12 + b32 * a22 + b33 * a32,
            b30 * a03 + b31 * a13 + b32 * a23 + b33 * a33
        )
    }
}

impl<T: Float> MulAssign for Mat4<T> {
    fn mul_assign(&mut self, other: Self) {
        let [a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31, a32, a33] = self.elements;
        let [b00, b01, b02, b03, b10, b11, b12, b13, b20, b21, b22, b23, b30, b31, b32, b33] = other.elements;

        self.elements[0] = b00 * a00 + b01 * a10 + b02 * a20 + b03 * a30;
        self.elements[1] = b00 * a01 + b01 * a11 + b02 * a21 + b03 * a31;
        self.elements[2] = b00 * a02 + b01 * a12 + b02 * a22 + b03 * a32;
        self.elements[3] = b00 * a03 + b01 * a13 + b02 * a23 + b03 * a33;
        self.elements[4] = b10 * a00 + b11 * a10 + b12 * a20 + b13 * a30;
        self.elements[5] = b10 * a01 + b11 * a11 + b12 * a21 + b13 * a31;
        self.elements[6] = b10 * a02 + b11 * a12 + b12 * a22 + b13 * a32;
        self.elements[7] = b10 * a03 + b11 * a13 + b12 * a23 + b13 * a33;
        self.elements[8] = b20 * a00 + b21 * a10 + b22 * a20 + b23 * a30;
        self.elements[9] = b20 * a01 + b21 * a11 + b22 * a21 + b23 * a31;
        self.elements[10] = b20 * a02 + b21 * a12 + b22 * a22 + b23 * a32;
        self.elements[11] = b20 * a03 + b21 * a13 + b22 * a23 + b23 * a33;
        self.elements[12] = b30 * a00 + b31 * a10 + b32 * a20 + b33 * a30;
        self.elements[13] = b30 * a01 + b31 * a11 + b32 * a21 + b33 * a31;
        self.elements[14] = b30 * a02 + b31 * a12 + b32 * a22 + b33 * a32;
        self.elements[15] = b30 * a03 + b31 * a13 + b32 * a23 + b33 * a33;
    }
}


impl<T: Float> Mul<Vec4<T>> for Mat4<T> {
    type Output = Vec4<T>;
    
    fn mul(self, other: Vec4<T>) -> Vec4<T> {
        let m = &self.elements;
        Vec4::new(
            m[0] * other.x() + m[4] * other.y() + m[8] * other.z() + m[12] * other.w(),
            m[1] * other.x() + m[5] * other.y() + m[9] * other.z() + m[13] * other.w(),
            m[2] * other.x() + m[6] * other.y() + m[10] * other.z() + m[14] * other.w(),
            m[3] * other.x() + m[7] * other.y() + m[11] * other.z() + m[15] * other.w()
        )
    }
}

#[cfg(test)]
mod tests {
    extern crate nalgebra_glm as glm;
    use super::*;

    #[test]
    fn tests() {

        // Rotation:
        let r1 = glm::rotate_x(&glm::rotate_y(&glm::rotate_z(&glm::Mat4::identity(), 2.4), 0.2), 1.1);
        let r2 = Mat4::from_rotation(Quat::from_euler(1.1, 0.2, 2.4));

        assert_eq!(r1.as_slice(), &r2[..]);

        let m1 = glm::Mat4::new_translation(&glm::vec3(0.0, 0.0, -5.0)) * glm::rotate_z(&glm::rotate_y(&glm::rotate_x(&glm::Mat4::identity(), 1.1), 0.2), 2.4) * glm::Mat4::new_scaling(1.0);
        let m2 = Mat4::from_translation_rotation_scale(Vec3::new(0.0, 0.0, -5.0), Quat::from_euler(1.1, 0.2, 2.4), Vec3::one());

        // From TRS
        assert_eq!(m1.as_slice(), &m2[..]);

        let s1 = glm::Mat4::new_translation(&glm::vec3(0.0, 0.0, 5.0));
        let s2 = Mat4::from_translation(Vec3::new(0.0, 0.0, 5.0));

        // Mul
        assert_eq!((m1 * s1).as_slice(), &(m2 * s2)[..]);

        let p1 = glm::perspective(4.0/3.0, 75.0, 0.01, 100.0);
        let p2 = Mat4::perspective(75.0, 4.0/3.0, 0.01, 100.0);

        // Proj
        assert_eq!((p1 * m1 * s1).as_slice(), &(p2 * m2 * s2)[..]);

        // Inverse
        assert_eq!((glm::inverse(&(p1 * m1 * s1))).as_slice(), &(p2 * m2 * s2).inverse()[..]);

        // As bytes
        assert_eq!(m2.as_bytes().len(), 16 * 4); // 32-bit
        assert_eq!(Mat4::<f64>::identity().as_bytes().len(), 16 * 8); // 64-bit

        assert_eq!(m2, Mat4::<f32>::from_bytes(&m2.as_bytes()));
    }
}