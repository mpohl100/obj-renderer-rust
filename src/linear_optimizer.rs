pub trait Objective
where
    Self: Sized + Clone,
{
    fn adjust(self, x: f64) -> Self
    where
        Self: Sized;
    /// @brief Evaluates the objective function
    fn evaluate(&self) -> f64;
}

pub struct LinearOptimizer<Obj: Objective> {
    marker: std::marker::PhantomData<Obj>,
    obj: Obj,
}

impl<Obj: Objective> LinearOptimizer<Obj> {
    pub fn new(obj: Obj) -> Self {
        LinearOptimizer {
            marker: std::marker::PhantomData,
            obj,
        }
    }

    pub fn optimize(&self, left_x: f64, right_x: f64, target: f64) -> Obj {
        let left_objective = Obj::adjust(self.obj.clone(), left_x);
        let right_objective = Obj::adjust(self.obj.clone(), right_x);
        let left_eval = left_objective.evaluate();
        let right_eval = right_objective.evaluate();
        let (start_x, start_step) =
            self.initialize_search(left_x, right_x, target, left_eval, right_eval);
        let mut current_x = start_x;
        let mut step = start_step;
        loop {
            let current_objective = Obj::adjust(self.obj.clone(), current_x);
            let current_eval = current_objective.evaluate();
            if (current_eval - target).abs() < 1e-6 {
                return current_objective;
            }
            if step >= 0.0 {
                if current_eval < target {
                    current_x += step;
                } else {
                    step /= 2.0;
                    current_x -= step;
                }
            } else {
                if current_eval > target {
                    current_x += step;
                } else {
                    step /= 2.0;
                    current_x -= step;
                }
            }
        }
    }

    fn initialize_search(
        &self,
        left_x: f64,
        right_x: f64,
        target: f64,
        left_eval: f64,
        right_eval: f64,
    ) -> (f64, f64) {
        let start_x;
        let start_step;
        if left_eval < target && right_eval < target {
            start_x = (left_x + right_x) / 2.0;
            start_step = (right_x - left_x) / 4.0;
        } else if left_eval > target && right_eval > target {
            start_x = (left_x + right_x) / 2.0;
            start_step = -(right_x - left_x) / 4.0;
        } else {
            if left_eval < right_eval {
                start_x = left_x;
                start_step = (right_x - left_x) / 4.0;
            } else {
                start_x = right_x;
                start_step = -(right_x - left_x) / 4.0;
            }
        }
        (start_x, start_step)
    }
}
