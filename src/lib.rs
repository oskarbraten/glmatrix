
mod vec3;
mod quat;

pub use vec3::Vec3;
pub use quat::Quat;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
