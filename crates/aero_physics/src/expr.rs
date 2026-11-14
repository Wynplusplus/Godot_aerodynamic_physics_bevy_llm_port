//! A small evaluator for the Godot `Expression` strings used by `AeroInfluencerControlAxisConfig`
//! (`expression_x`, `expression_y`, `expression_z`).
//!
//! Supported: numbers, string literals, unary and binary `+ - * /`, parentheses, and the functions
//! `clamp`, `remap`, `min`, `max`, `abs`, `sign`, `pow`, `sqrt`, `lerp`, `deg_to_rad`, `rad_to_deg`,
//! and `get_control_command("axis")`.

#[derive(Debug, Clone, PartialEq)]
enum Node {
    Number(f32),
    Text(String),
    Neg(Box<Node>),
    Bin(char, Box<Node>, Box<Node>),
    Call(String, Vec<Node>),
}

/// A parsed expression.
#[derive(Debug, Clone, PartialEq)]
pub struct Expr(Node);

/// Something that can answer `get_control_command(name)` lookups while an expression runs.
pub trait ControlSource {
    fn control_command(&self, axis: &str) -> f32;
}

impl Expr {
    pub fn parse(source: &str) -> Result<Self, String> {
        let mut parser = Parser { chars: source.chars().collect(), pos: 0 };
        let node = parser.expression()?;
        parser.skip_ws();
        if parser.pos != parser.chars.len() {
            return Err(format!("unexpected character '{}' in expression", parser.chars[parser.pos]));
        }
        Ok(Self(node))
    }

    pub fn eval(&self, source: &dyn ControlSource) -> f32 {
        eval_node(&self.0, source)
    }
}

fn eval_node(node: &Node, src: &dyn ControlSource) -> f32 {
    match node {
        Node::Number(v) => *v,
        // Strings only appear as arguments to get_control_command; they evaluate to 0 elsewhere.
        Node::Text(_) => 0.0,
        Node::Neg(inner) => -eval_node(inner, src),
        Node::Bin(op, a, b) => {
            let (x, y) = (eval_node(a, src), eval_node(b, src));
            match op {
                '+' => x + y,
                '-' => x - y,
                '*' => x * y,
                _ => x / y,
            }
        }
        Node::Call(name, args) => {
            if name == "get_control_command" {
                if let Some(Node::Text(axis)) = args.first() {
                    return src.control_command(axis);
                }
                return 0.0;
            }
            let v: Vec<f32> = args.iter().map(|a| eval_node(a, src)).collect();
            let arg = |i: usize| v.get(i).copied().unwrap_or(0.0);
            match name.as_str() {
                "clamp" => arg(0).clamp(arg(1), arg(2)),
                "remap" => crate::godot::remap(arg(0), arg(1), arg(2), arg(3), arg(4)),
                "min" => arg(0).min(arg(1)),
                "max" => arg(0).max(arg(1)),
                "abs" => arg(0).abs(),
                "sign" => crate::godot::sign(arg(0)),
                "pow" => arg(0).powf(arg(1)),
                "sqrt" => arg(0).sqrt(),
                "lerp" => arg(0) + (arg(1) - arg(0)) * arg(2),
                "deg_to_rad" => arg(0).to_radians(),
                "rad_to_deg" => arg(0).to_degrees(),
                _ => 0.0,
            }
        }
    }
}

struct Parser {
    chars: Vec<char>,
    pos: usize,
}

impl Parser {
    fn skip_ws(&mut self) {
        while self.pos < self.chars.len() && self.chars[self.pos].is_whitespace() {
            self.pos += 1;
        }
    }

    fn peek(&mut self) -> Option<char> {
        self.skip_ws();
        self.chars.get(self.pos).copied()
    }

    fn expression(&mut self) -> Result<Node, String> {
        let mut lhs = self.term()?;
        while let Some(op @ ('+' | '-')) = self.peek() {
            self.pos += 1;
            let rhs = self.term()?;
            lhs = Node::Bin(op, Box::new(lhs), Box::new(rhs));
        }
        Ok(lhs)
    }

    fn term(&mut self) -> Result<Node, String> {
        let mut lhs = self.unary()?;
        while let Some(op @ ('*' | '/')) = self.peek() {
            self.pos += 1;
            let rhs = self.unary()?;
            lhs = Node::Bin(op, Box::new(lhs), Box::new(rhs));
        }
        Ok(lhs)
    }

    fn unary(&mut self) -> Result<Node, String> {
        if self.peek() == Some('-') {
            self.pos += 1;
            return Ok(Node::Neg(Box::new(self.unary()?)));
        }
        self.primary()
    }

    fn primary(&mut self) -> Result<Node, String> {
        match self.peek() {
            Some('(') => {
                self.pos += 1;
                let inner = self.expression()?;
                self.expect(')')?;
                Ok(inner)
            }
            Some('"') => {
                self.pos += 1;
                let start = self.pos;
                while self.pos < self.chars.len() && self.chars[self.pos] != '"' {
                    self.pos += 1;
                }
                let text: String = self.chars[start..self.pos].iter().collect();
                self.expect('"')?;
                Ok(Node::Text(text))
            }
            Some(c) if c.is_ascii_digit() || c == '.' => {
                let start = self.pos;
                while self.pos < self.chars.len() && (self.chars[self.pos].is_ascii_digit() || self.chars[self.pos] == '.') {
                    self.pos += 1;
                }
                let text: String = self.chars[start..self.pos].iter().collect();
                text.parse::<f32>().map(Node::Number).map_err(|e| e.to_string())
            }
            Some(c) if c.is_ascii_alphabetic() || c == '_' => {
                let start = self.pos;
                while self.pos < self.chars.len() && (self.chars[self.pos].is_ascii_alphanumeric() || self.chars[self.pos] == '_') {
                    self.pos += 1;
                }
                let name: String = self.chars[start..self.pos].iter().collect();
                self.expect('(')?;
                let mut args = Vec::new();
                if self.peek() != Some(')') {
                    loop {
                        args.push(self.expression()?);
                        if self.peek() == Some(',') {
                            self.pos += 1;
                        } else {
                            break;
                        }
                    }
                }
                self.expect(')')?;
                Ok(Node::Call(name, args))
            }
            other => Err(format!("unexpected token {other:?} in expression")),
        }
    }

    fn expect(&mut self, c: char) -> Result<(), String> {
        if self.peek() == Some(c) {
            self.pos += 1;
            Ok(())
        } else {
            Err(format!("expected '{c}' in expression"))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Ctl;
    impl ControlSource for Ctl {
        fn control_command(&self, axis: &str) -> f32 {
            if axis == "rotor_tilt" { -0.15 } else { 0.0 }
        }
    }

    #[test]
    fn evaluates_tilt_expression() {
        let e = Expr::parse("clamp(remap(get_control_command(\"rotor_tilt\"), -0.3, 0, 0, 0.1), 0.0,  0.1)").unwrap();
        assert!((e.eval(&Ctl) - 0.05).abs() < 1e-6);
    }

    #[test]
    fn arithmetic_precedence() {
        let e = Expr::parse("1 + 2 * 3 - -4 / 2").unwrap();
        assert!((e.eval(&Ctl) - 9.0).abs() < 1e-6);
    }
}
