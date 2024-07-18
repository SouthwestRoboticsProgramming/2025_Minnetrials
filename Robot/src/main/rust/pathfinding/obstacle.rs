use std::f64::consts::PI;

use lerp::Lerp;

use super::{
    geom::{Arc, EnvPolygon, Segment},
    math::{self, Vec2f},
};

pub struct Circle {
    pub position: Vec2f,
    pub radius: f64,
}

pub struct Polygon {
    pub vertices: Vec<Vec2f>,
}

pub enum Obstacle {
    Circle(Circle),
    Polygon(Polygon),
}

impl Obstacle {
    // pub fn convert_into(&self, inflate: f64, arcs: &mut Vec<Arc>, segments: &mut Vec<Segment>) {
    //     match self {
    //         Self::Circle(c) => convert_circle(c, inflate, arcs),
    //         Self::Polygon(p) => convert_polygon(p, inflate, arcs, segments),
    //     };
    // }
    pub fn convert_into_env(&self, inflate: f64) -> EnvPolygon {
        match self {
            Obstacle::Circle(c) => convert_circle(c, inflate),
            Obstacle::Polygon(p) => convert_polygon(p, inflate),
        }
    }
}

fn convert_circle(circle: &Circle, inflate: f64) -> EnvPolygon {
    // arcs.push(Arc {
    //     center: circle.position,
    //     radius: circle.radius + inflate,
    //     min_angle: 0.0,
    //     max_angle: 0.0,
    // });
    EnvPolygon {
        arcs: vec![Arc {
            center: circle.position,
            radius: circle.radius + inflate,
            min_angle: 0.0,
            max_angle: 0.0,
        }],
        segments: vec![],
        inverted: false,
    }
}

fn convert_polygon(polygon: &Polygon, inflate: f64) -> EnvPolygon {
    let size = polygon.vertices.len();

    let mut segments = Vec::new();
    let mut arcs = Vec::new();

    let mut prev = polygon.vertices[size - 1];
    for i in 0..size {
        let vertex = polygon.vertices[i];
        let delta = vertex - prev;

        let scale = inflate / delta.length();
        let offset = Vec2f {
            x: delta.y * scale,
            y: -delta.x * scale,
        };

        segments.push(Segment {
            from: vertex + offset,
            to: prev + offset,
        });

        let next = polygon.vertices[(i + 1) % size];
        let edge_angle = delta.angle();
        let next_angle = (next - vertex).angle();

        // Only add arc if the vertex is convex
        if math::floor_mod(next_angle - edge_angle, PI * 2.0) < PI {
            arcs.push(Arc::new(
                vertex,
                inflate,
                edge_angle - PI / 2.0,
                next_angle - PI / 2.0,
            ));
        }

        prev = vertex;
    }

    // Concave vertices will generate overlapping segments, so we need to
    // remove those overlaps
    for i in 0..size {
        let i2 = (i + 1) % size;

        let a = &segments[i];
        let b = &segments[i2];

        let d1 = a.to - a.from;
        let d2 = b.to - b.from;

        let vp = d1.x * d2.y - d2.x * d1.y;
        let v = b.from - a.from;

        let k1 = (v.x * d2.y - v.y * d2.x) / vp;
        let k2 = (v.x * d1.y - v.y * d1.x) / vp;

        if vp != 0.0 && 0.0 <= k1 && k1 <= 1.0 && 0.0 <= k2 && k2 <= 1.0 {
            // Segments overlap, clip them
            let intersect = a.from.lerp(a.to, k1);
            segments[i].from = intersect;
            segments[i2].to = intersect;
        }
    }

    // Check winding order
    let first = polygon.vertices[0];
    let last = polygon.vertices.last().unwrap();
    let mut total = last.x * first.y - first.x * last.y;
    for i in 0..(size - 1) {
        let a = polygon.vertices[i];
        let b = polygon.vertices[i + 1];
        total += a.x * b.y - b.x * a.y;
    }
    let inverted = total < 0.0;

    EnvPolygon {
        arcs,
        segments,
        inverted,
    }
}
