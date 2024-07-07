use std::f64::consts::PI;

use super::{
    geom::{Arc, Segment},
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
    pub fn convert_into(&self, inflate: f64, arcs: &mut Vec<Arc>, segments: &mut Vec<Segment>) {
        match self {
            Self::Circle(c) => convert_circle(c, inflate, arcs),
            Self::Polygon(p) => convert_polygon(p, inflate, arcs, segments),
        };
    }
}

fn convert_circle(circle: &Circle, inflate: f64, arcs: &mut Vec<Arc>) {
    arcs.push(Arc {
        center: circle.position,
        radius: circle.radius + inflate,
        min_angle: 0.0,
        max_angle: 0.0,
    });
}

fn convert_polygon(
    polygon: &Polygon,
    inflate: f64,
    arcs: &mut Vec<Arc>,
    segments: &mut Vec<Segment>,
) {
    let size = polygon.vertices.len();

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
}
