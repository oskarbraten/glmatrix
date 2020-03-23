use std::convert::From;
use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign};
use super::{Vec3, Vec4, Quat};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Mat4 {
    pub elements: [f32; 16]
}

impl Mat4 {
    pub const fn new(m00: f32, m01: f32, m02: f32, m03: f32, m10: f32, m11: f32, m12: f32, m13: f32, m20: f32, m21: f32, m22: f32, m23: f32, m30: f32, m31: f32, m32: f32, m33: f32) -> Self {
        Self {
            elements: [m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33]
        }
    }

    pub const fn identity() -> Self {
        let mut m = Self {
            elements: [0.0; 16]
        };
        
        m.elements[0] = 1.0;
        m.elements[5] = 1.0;
        m.elements[10] = 1.0;
        m.elements[15] = 1.0;
        
        m
    }
    
    pub fn transpose(&self) -> Self {
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
        
    pub fn determinant(&self) -> f32 {
        let [a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31, a32, a33] = &self.elements;
        
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
        let [a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31, a32, a33] = &self.elements;
        
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
        
        let inverse_det = 1.0 / (b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06);
        
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
        let [a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31, a32, a33] = &self.elements;
    
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

    pub fn translate(mut self, v: Vec3) -> Self {
        let [x, y, z] = &v.elements;

        self.elements[12] = self.elements[0] * x + self.elements[4] * y + self.elements[8] * z + self.elements[12];
        self.elements[13] = self.elements[1] * x + self.elements[5] * y + self.elements[9] * z + self.elements[13];
        self.elements[14] = self.elements[2] * x + self.elements[6] * y + self.elements[10] * z + self.elements[14];
        self.elements[15] = self.elements[3] * x + self.elements[7] * y + self.elements[11] * z + self.elements[15];

        self
    }

    pub fn scale(mut self, v: Vec3) -> Self {
        let [x, y, z] = &v.elements;

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

        self
    }

    pub fn get_translation(&self) -> Vec3 {
        Vec3::new(self.elements[12], self.elements[13], self.elements[14])
    }

    pub fn get_scaling(&self) -> Vec3 {
        Vec3::new(
            (self.elements[0].powi(2) + self.elements[2].powi(2) + self.elements[3].powi(2)).sqrt(),
            (self.elements[4].powi(2) + self.elements[5].powi(2) + self.elements[6].powi(2)).sqrt(),
            (self.elements[8].powi(2) + self.elements[9].powi(2) + self.elements[10].powi(2)).sqrt(),
        )
    }

    pub fn get_rotation(&self) -> Quat {
        unimplemented!()
    }

    pub fn from_rotation(r: Quat) -> Self {
        let [x, y, z, w] = &r.elements;
    
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
            1.0 - yy - zz,
            yx + wz,
            zx - wy,
            0.0,
            yx - wz,
            1.0 - xx - zz,
            zy + wx,
            0.0,
            zx + wy,
            zy - wx,
            1.0 - xx - yy,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0
        )
    }

    pub fn from_translation(t: Vec3) -> Self {
        Self::new(
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            t.x(),
            t.y(),
            t.z(),
            1.0
        )
    }

    pub fn from_scaling(s: Vec3) -> Self {
        Self::new(
            s.x(),
            0.0,
            0.0,
            0.0,
            0.0,
            s.y(),
            0.0,
            0.0,
            0.0,
            0.0,
            s.z(),
            0.0,
            0.0,
            0.0,
            0.0,
            1.0
        )
    }

    pub fn from_rotation_translation_scale(r: Quat, t: Vec3, s: Vec3) -> Self {
        let [x, y, z, w] = &r.elements;
    
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
            (1.0 - (yy + zz)) * sx,
            (xy + wz) * sx,
            (xz - wy) * sx,
            0.0,
            (xy - wz) * sy,
            (1.0 - (xx + zz)) * sy,
            (yz + wx) * sy,
            0.0,
            (xz + wy) * sz,
            (yz - wx) * sz,
            (1.0 - (xx + yy)) * sz,
            0.0,
            t.x(),
            t.y(),
            t.z(),
            1.0
        )
    }

    pub fn perspective(fovy: f32, aspect: f32, near: f32, far: f32) -> Self {
        let f = 1.0 / (fovy / 2.0).tan();
        let mut m = Self::new(
            f / aspect,
            0.0,
            0.0,
            0.0,
            0.0,
            f,
            0.0,
            0.0,
            0.0,
            0.0,
            -1.0,
            -1.0,
            0.0,
            0.0,
            -2.0 * near,
            0.0
        );
    
        if far != std::f32::INFINITY {
            let nf = 1.0 / (near - far);
            m.elements[10] = (far + near) * nf;
            m.elements[14] = 2.0 * far * near * nf;
        }

        m
    }

    pub fn ortho(left: f32, right: f32, bottom: f32, top: f32, near: f32, far: f32) -> Self {
        let lr = 1.0 / (left - right);
        let bt = 1.0 / (bottom - top);
        let nf = 1.0 / (near - far);

        Self::new(
            -2.0 * lr,
            0.0,
            0.0,
            0.0,
            0.0,
            -2.0 * bt,
            0.0,
            0.0,
            0.0,
            0.0,
            2.0 * nf,
            0.0,
            (left + right) * lr,
            (top + bottom) * bt,
            (far + near) * nf,
            1.0
        )
    }


}

impl From<[f32; 16]> for Mat4 {
    fn from(elements: [f32; 16]) -> Self {
        Self {
            elements
        }
    }
}

impl Add for Mat4 {
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

impl AddAssign for Mat4 {
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

impl Sub for Mat4 {
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

impl SubAssign for Mat4 {
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

impl Mul for Mat4 {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        let [a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31, a32, a33] = &self.elements;
        let [b00, b01, b02, b03, b10, b11, b12, b13, b20, b21, b22, b23, b30, b31, b32, b33] = &other.elements;

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

impl MulAssign for Mat4 {
    fn mul_assign(&mut self, other: Self) {
        let [a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31, a32, a33] = self.elements;
        let [b00, b01, b02, b03, b10, b11, b12, b13, b20, b21, b22, b23, b30, b31, b32, b33] = &other.elements;

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


impl Mul<Vec4> for Mat4 {
    type Output = Vec4;
    
    fn mul(self, other: Vec4) -> Vec4 {
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
    use super::*;

    const mat_a: Mat4 = Mat4::identity();



    #[test]
    fn transpose() {

        

        assert_eq!(2 + 2, 4);
    }
}