use na::Vector2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Movement {
    Stop,
    Up,
    UpRight,
    Right,
    DownRight,
    Down,
    DownLeft,
    Left,
    UpLeft,
}

impl Into<Vector2<f32>> for Movement {
    fn into(self) -> Vector2<f32> {
        let (dir_x, dir_y): (f32, f32) = match self {
            Movement::Up => (0., -1.),
            Movement::UpRight => (1., -1.),
            Movement::Right => (1., 0.),
            Movement::DownRight => (1., 1.),
            Movement::Down => (0., 1.),
            Movement::DownLeft => (-1., 1.),
            Movement::Left => (-1., 0.),
            Movement::UpLeft => (-1., -1.),
            Movement::Stop => {
                return Vector2::new(0., 0.);
            }
        };
        Vector2::new(dir_x, dir_y).normalize()
    }
}
