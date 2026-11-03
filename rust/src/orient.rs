use drivers::types::Raw3;

pub fn orient(raw: Raw3, bias: Raw3, axes: [usize; 3], signs: [i32; 3]) -> [i32; 3] {
    let corrected = [
        raw.x() as i32 - bias.x() as i32,
        raw.y() as i32 - bias.y() as i32,
        raw.z() as i32 - bias.z() as i32,
    ];
    return [
        corrected[axes[0]] * signs[0],
        corrected[axes[1]] * signs[1],
        corrected[axes[2]] * signs[2],
    ]

}
