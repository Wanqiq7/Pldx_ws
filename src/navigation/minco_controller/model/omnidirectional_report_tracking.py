"""
全向移动机器人 MPC 轨迹跟踪：中科大哨兵 2025 技术报告模型

=============================================================================
模型与 QP（报告 5.5.5.3）
=============================================================================
状态: x = [p_x, p_y, ψ, v_x, v_y, ω]ᵀ
输入: u = [F_x, F_y, M_z]ᵀ，坐标均采用世界坐标系
离散模型: x[k+1] = A x[k] + B u[k]，使用报告中的前向欧拉矩阵
预测序列: X = [x[1], ..., x[N]]ᵀ = A_bar x[0] + B_bar U
目标函数: (X-X_ref)ᵀ Q_bar (X-X_ref) + Uᵀ R_bar U
约束: -F_max ≤ F_x,F_y ≤ F_max，-M_max ≤ M_z ≤ M_max，
并可设置预测状态分量的上下限。使用 qpOASES 求解完整的稠密 QP。

注意：报告第 23 页将绝对值约束展开为不等式时，下半部分右端项的
负号与其前面的绝对值公式矛盾。这里按绝对值公式实现正确的双边约束。
报告未给出具体的质量、惯量、权重和限值；main 中的数值仅用于仿真。

=============================================================================
示例范围（报告 5.5.5.4）
=============================================================================
复用 omnidirectional_dynamic_tracking.py 的 8 字轨迹替代 MINCO：
近似投影、首次求解、二次采样、再次求解，
并插值提取 10 ms 处的预测速度。示例假设下位机完美跟踪速度，角速度
也取 MPC 预测值。传感器时间同步、真实 MINCO 接入、ESDF 及底盘旋转
模式不属于本文件，不能据此宣称完整复现报告中的机器人控制器。

依赖: numpy、scipy、casadi（需包含 qpOASES 插件）、acados_template、
matplotlib（轨迹生成器所在的原脚本会导入 acados 和 matplotlib）。
"""

from dataclasses import dataclass
from time import perf_counter
from typing import Optional, Tuple

import casadi as ca
import numpy as np
from scipy.optimize import minimize_scalar

from omnidirectional_dynamic_tracking import DynamicTrajectoryGenerator as SourceTrajectoryGenerator


# =============================================================================
# 轨迹生成器：复用原脚本的 8 字轨迹，接口可替换成 MINCO 位置和速度
# =============================================================================
class DynamicTrajectoryGenerator:
    """将原始 8 字轨迹适配为报告模型需要的二维状态和有限时域。"""

    def __init__(
        self,
        scale: float = 1.5,
        period: float = 10.0,
        center: Tuple[float, float] = (0.0, 0.0),
        duration: float = 10.0,
    ):
        if not all(np.isfinite([scale, period, duration, *center])):
            raise ValueError('轨迹参数必须为有限数值')
        if scale <= 0.0 or period <= 0.0 or duration <= 0.0:
            raise ValueError('scale、period 和 duration 必须大于零')
        self.period = float(period)
        self.duration = float(duration)
        self.source = SourceTrajectoryGenerator(
            trajectory_type='figure8', scale=scale, period=period, center=center,
        )

    def get_position(self, t: float) -> np.ndarray:
        """返回 t 时刻的 [p_x, p_y]，单位为 m。"""
        return np.asarray(self.source.get_position(t)[:2])

    def get_velocity(self, t: float) -> np.ndarray:
        """返回 t 时刻的世界坐标系速度 [v_x, v_y]，单位为 m/s。"""
        return np.asarray(self.source.get_velocity(t)[:2])

    def get_full_state(self, t: float) -> np.ndarray:
        """返回原始轨迹的位置、切向航向、速度和切向角速度。"""
        return self.source.get_full_state(t)

    def project_time(
        self, position: np.ndarray, previous_time: Optional[float] = None, samples: int = 201,
    ) -> float:
        """粗采样并细化投影；提供上一投影时只搜索其附近，避免自交处分支跳变。

        这是 8 字示例的连续性约定；真实 MINCO 的投影应按轨迹段实现。
        """
        position = np.asarray(position, dtype=float)
        if position.shape != (2,) or not np.isfinite(position).all() or samples < 3:
            raise ValueError('position 应为有限二维向量，samples 至少为 3')
        if previous_time is not None:
            if not np.isfinite(previous_time) or not 0.0 <= previous_time <= self.duration:
                raise ValueError('previous_time 必须位于轨迹时域内')
            search_start = max(0.0, previous_time - self.period / 4.0)
            search_end = min(self.duration, previous_time + self.period / 4.0)
        else:
            search_start, search_end = 0.0, self.duration
        times = np.linspace(search_start, search_end, samples)

        def distance_squared(t: float) -> float:
            delta = self.get_position(t) - position
            return float(delta @ delta)

        distances = np.array([distance_squared(t) for t in times])
        candidates = [search_start, search_end]
        for i in range(1, samples - 1):
            if distances[i] <= distances[i - 1] and distances[i] <= distances[i + 1]:
                result = minimize_scalar(
                    distance_squared, bounds=(times[i - 1], times[i + 1]), method='bounded'
                )
                if result.success:
                    candidates.append(float(result.x))
        return min(candidates, key=distance_squared)


def sample_reference_states(
    trajectory: DynamicTrajectoryGenerator,
    times: np.ndarray,
    initial_yaw: float,
    omega_ref: Optional[float] = None,
    velocity_threshold: float = 1e-8,
) -> np.ndarray:
    """生成 N×6 参考状态；默认采用原轨迹的切向角速度。

    报告未定义轨迹终点之后的补点和零速度航向：这里采用终点停住，
    零速度时保持上一航向。将角度展开到连续分支，避免 ±π 人为跳变。
    """
    times = np.asarray(times, dtype=float)
    if times.ndim != 1 or not np.isfinite(times).all():
        raise ValueError('times 必须是一维有限时间数组')
    states = np.zeros((len(times), 6))
    last_yaw = float(initial_yaw)
    for i, time in enumerate(times):
        t = float(np.clip(time, 0.0, trajectory.duration))
        source_state = trajectory.get_full_state(t)
        position = source_state[:2]
        velocity = source_state[3:5] if time <= trajectory.duration else np.zeros(2)
        if np.linalg.norm(velocity) > velocity_threshold:
            raw_yaw = np.arctan2(velocity[1], velocity[0])
            last_yaw += np.arctan2(np.sin(raw_yaw - last_yaw), np.cos(raw_yaw - last_yaw))
        angular_velocity = source_state[5] if time <= trajectory.duration else 0.0
        if omega_ref is not None:
            angular_velocity = omega_ref
        states[i] = [position[0], position[1], last_yaw, velocity[0], velocity[1], angular_velocity]
    return states


# =============================================================================
# 二次参考轨迹选取（报告 5.5.5.4.3）
# =============================================================================
class SecondaryResamplingStrategy:
    """根据首次预测和首次参考的对应速度，缩短参考轨迹的时间间隔。"""

    def __init__(self, dt: float, velocity_threshold: float = 1e-8):
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError('dt 必须为正的有限数值')
        if not np.isfinite(velocity_threshold) or velocity_threshold <= 0.0:
            raise ValueError('velocity_threshold 必须为正的有限数值')
        self.dt = float(dt)
        self.velocity_threshold = float(velocity_threshold)

    def compute_alphas(self, predicted_states: np.ndarray, references: np.ndarray) -> np.ndarray:
        """α_i = max(0, cos(v_prim,i, v_ref,i))。

        报告的零速度情形未定义；分母过小时约定 α=1。这里 v_ref,i 取
        首次参考轨迹的对应速度，避免将重采样时间与 α 相互循环定义。
        """
        predicted_states = np.asarray(predicted_states, dtype=float)
        references = np.asarray(references, dtype=float)
        if (predicted_states.ndim != 2 or predicted_states.shape[1] != 6
                or predicted_states.shape != references.shape
                or not np.isfinite(predicted_states).all()
                or not np.isfinite(references).all()):
            raise ValueError('预测和参考状态必须是同形状的有限 N×6 数组')
        v_prim = predicted_states[:, 3:5]
        v_ref = references[:, 3:5]
        norm_prim = np.linalg.norm(v_prim, axis=1)
        norm_ref = np.linalg.norm(v_ref, axis=1)
        valid = (norm_prim > self.velocity_threshold) & (norm_ref > self.velocity_threshold)
        alphas = np.ones(len(predicted_states))
        alphas[valid] = np.clip(
            np.sum(v_prim[valid] * v_ref[valid], axis=1) / (norm_prim[valid] * norm_ref[valid]),
            0.0, 1.0,
        )
        return alphas

    def resample_trajectory(
        self, trajectory: DynamicTrajectoryGenerator, t_start: float,
        predicted_states: np.ndarray, first_references: np.ndarray, initial_yaw: float,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """按报告 t_i=t_start+Σ_{j=0}^i(dt·α_j) 生成二次参考。"""
        alphas = self.compute_alphas(predicted_states, first_references)
        times = t_start + self.dt * np.cumsum(alphas)
        references = sample_reference_states(trajectory, times, initial_yaw)
        return references, times, alphas


# =============================================================================
# 报告的离散模型及稠密 QP
# =============================================================================
@dataclass(frozen=True)
class ReportMpcConfig:
    """物理参数、代价及约束；全部值都应根据实际机器人确定。

    state_lower/state_upper 的顺序为 [p_x,p_y,ψ,v_x,v_y,ω]；可使用 ±inf
    表示不限制该状态。Q 和 R 的对角元素对应单个预测步的状态和输入。
    """

    mass: float
    inertia_z: float
    dt: float
    horizon_steps: int
    q_diag: np.ndarray
    r_diag: np.ndarray
    force_max: float
    moment_max: float
    state_lower: np.ndarray
    state_upper: np.ndarray


@dataclass
class MpcSolution:
    """预测状态为 [x_1,...,x_N]，输入为 [u_0,...,u_{N-1}]。

    solve_time 包含一次 qpOASES 调用和参数传递，不含参考生成和状态还原。
    """

    states: np.ndarray
    controls: np.ndarray
    solve_time: float


def build_discrete_model(mass: float, inertia_z: float, dt: float) -> Tuple[np.ndarray, np.ndarray]:
    """原样构造报告第 22 页的 A 和 B，无 0.5*dt² 位置项。"""
    if not np.isfinite([mass, inertia_z, dt]).all() or min(mass, inertia_z, dt) <= 0.0:
        raise ValueError('质量、转动惯量和时间步长必须为正的有限数值')
    A = np.eye(6)
    A[0, 3] = A[1, 4] = A[2, 5] = dt
    B = np.zeros((6, 3))
    B[3, 0] = B[4, 1] = dt / mass
    B[5, 2] = dt / inertia_z
    return A, B


def build_prediction_matrices(A: np.ndarray, B: np.ndarray, N: int) -> Tuple[np.ndarray, np.ndarray]:
    """构造 X=A_bar*x0+B_bar*U，行块依次对应 x_1,...,x_N。"""
    if A.shape != (6, 6) or B.shape != (6, 3) or N <= 0:
        raise ValueError('A、B 的维数应为 6×6、6×3，N 必须大于零')
    powers = [np.eye(6)]
    for _ in range(N):
        powers.append(powers[-1] @ A)
    A_bar = np.vstack(powers[1:])
    B_bar = np.zeros((6 * N, 3 * N))
    for i in range(N):
        for j in range(i + 1):
            B_bar[6*i:6*(i+1), 3*j:3*(j+1)] = powers[i-j] @ B
    return A_bar, B_bar


class ReportMpcSolver:
    """预构造报告的完整稠密 QP，在线更新梯度与边界。"""

    def __init__(self, config: ReportMpcConfig):
        self.config = config
        if not isinstance(config.horizon_steps, int) or config.horizon_steps <= 0:
            raise ValueError('horizon_steps 必须为正整数')
        if not np.isfinite([config.force_max, config.moment_max]).all() or min(config.force_max, config.moment_max) <= 0:
            raise ValueError('力和力矩上限必须为正的有限数值')
        q = np.asarray(config.q_diag, dtype=float)
        r = np.asarray(config.r_diag, dtype=float)
        lower = np.asarray(config.state_lower, dtype=float)
        upper = np.asarray(config.state_upper, dtype=float)
        if (q.shape != (6,) or r.shape != (3,) or not np.isfinite(q).all()
                or not np.isfinite(r).all() or np.any(q < 0.0) or np.any(r <= 0.0)):
            raise ValueError('Q 对角线须非负，R 对角线须为正，且维数分别为 6 和 3')
        if (lower.shape != (6,) or upper.shape != (6,) or np.isnan(lower).any()
                or np.isnan(upper).any() or np.any(lower > upper)
                or np.isposinf(lower).any() or np.isneginf(upper).any()):
            raise ValueError('状态上下界无效')
        self.N = config.horizon_steps
        self.A, self.B = build_discrete_model(config.mass, config.inertia_z, config.dt)
        self.A_bar, self.B_bar = build_prediction_matrices(self.A, self.B, self.N)
        self.Q_bar = np.diag(np.tile(q, self.N))
        R_bar = np.diag(np.tile(r, self.N))
        self.BtQ = self.B_bar.T @ self.Q_bar
        self.H = 2.0 * (self.BtQ @ self.B_bar + R_bar)
        self.H = 0.5 * (self.H + self.H.T)

        # qpOASES 使用双边约束：lbx≤U≤ubx，lba≤A_qp·U≤uba。
        # 这与报告的 G U≤g 等价；只保留有限上下界对应的状态行。
        mask = np.tile(np.isfinite(lower) | np.isfinite(upper), self.N)
        self.A_qp = self.B_bar[mask, :]
        self.state_rows = np.flatnonzero(mask)
        self.state_lower = np.tile(lower, self.N)[mask]
        self.state_upper = np.tile(upper, self.N)[mask]
        force_bounds = np.array([config.force_max, config.force_max, config.moment_max])
        self.input_upper = np.tile(force_bounds, self.N)
        self.input_lower = -self.input_upper

        # Hessian、约束矩阵和输入边界不随控制周期改变，预先转换以避免重复拷贝。
        self._H_dm = ca.DM(self.H)
        self._A_qp_dm = ca.DM(self.A_qp)
        self._input_lower_dm = ca.DM(self.input_lower)
        self._input_upper_dm = ca.DM(self.input_upper)
        self.solver = ca.conic(
            'report_mpc', 'qpoases',
            {'h': ca.Sparsity.dense(3*self.N, 3*self.N),
             'a': ca.Sparsity.dense(self.A_qp.shape[0], 3*self.N)},
            {'printLevel': 'none', 'print_time': False, 'error_on_fail': False,
             'initialStatusBounds': 'inactive'},
        )

    def solve(self, initial_state: np.ndarray, references: np.ndarray) -> MpcSolution:
        """求解一次 QP；references 的第 i 行对应预测状态 x_(i+1)。"""
        initial_state = np.asarray(initial_state, dtype=float)
        references = np.asarray(references, dtype=float)
        if (initial_state.shape != (6,) or references.shape != (self.N, 6)
                or not np.isfinite(initial_state).all() or not np.isfinite(references).all()):
            raise ValueError('初始状态须为有限 6 向量，参考须为有限 N×6 数组')
        base = self.A_bar @ initial_state
        gradient = 2.0 * self.BtQ @ (base - references.reshape(-1))
        constrained_base = base[self.state_rows]
        start = perf_counter()
        result = self.solver(
            h=self._H_dm, g=gradient, a=self._A_qp_dm,
            lbx=self._input_lower_dm, ubx=self._input_upper_dm,
            lba=self.state_lower - constrained_base,
            uba=self.state_upper - constrained_base,
        )
        elapsed = perf_counter() - start
        stats = self.solver.stats()
        if not stats.get('success', False):
            raise RuntimeError(f"qpOASES 求解失败: {stats.get('return_status', '未知状态')}")
        controls = np.asarray(result['x']).reshape(self.N, 3)
        states = (base + self.B_bar @ controls.reshape(-1)).reshape(self.N, 6)
        return MpcSolution(states=states, controls=controls, solve_time=elapsed)


def solve_two_pass_tracking(
    solver: ReportMpcSolver, trajectory: DynamicTrajectoryGenerator,
    initial_state: np.ndarray, t_start: float,
) -> Tuple[MpcSolution, MpcSolution, np.ndarray, np.ndarray, np.ndarray]:
    """从投影时间开始，先均匀采样求解，再根据首次结果二次采样求解。"""
    times = t_start + solver.config.dt * np.arange(solver.N)
    first_ref = sample_reference_states(trajectory, times, initial_state[2])
    first = solver.solve(initial_state, first_ref)
    resampler = SecondaryResamplingStrategy(solver.config.dt)
    second_ref, second_times, alphas = resampler.resample_trajectory(
        trajectory, t_start, first.states, first_ref, initial_state[2]
    )
    second = solver.solve(initial_state, second_ref)
    return first, second, first_ref, second_times, alphas


def predicted_velocity_at(
    initial_state: np.ndarray, predicted_states: np.ndarray, dt: float, lookahead: float = 0.01,
) -> np.ndarray:
    """线性插值得到预测速度 [v_x,v_y,ω]；报告指定平移前瞻 10 ms。

    ω 在示例中也按相同方式取得，仅用于独立仿真，并非报告的旋转模式。
    """
    initial_state = np.asarray(initial_state, dtype=float)
    predicted_states = np.asarray(predicted_states, dtype=float)
    if (initial_state.shape != (6,) or predicted_states.ndim != 2
            or predicted_states.shape[1] != 6 or len(predicted_states) == 0
            or not np.isfinite(initial_state).all() or not np.isfinite(predicted_states).all()
            or not np.isfinite([dt, lookahead]).all() or dt <= 0.0
            or lookahead < 0.0 or lookahead > len(predicted_states) * dt):
        raise ValueError('状态、时间步长或前瞻时间无效')
    times = np.arange(len(predicted_states) + 1) * dt
    velocities = np.vstack((initial_state[3:6], predicted_states[:, 3:6]))
    return np.array([np.interp(lookahead, times, velocities[:, j]) for j in range(3)])


# =============================================================================
# 简化闭环仿真与可视化
# =============================================================================
def run_tracking_simulation(
    solver: ReportMpcSolver, trajectory: DynamicTrajectoryGenerator,
    x0: np.ndarray, simulation_time: float = 2.0, control_period: float = 0.01,
) -> dict:
    """模拟 100 Hz 速度控制；假设底层能在每周期末完美达到速度命令。"""
    if (not np.isfinite([simulation_time, control_period]).all()
            or simulation_time <= 0.0 or control_period <= 0.0):
        raise ValueError('仿真时长和控制周期必须为正的有限数值')
    steps_float = simulation_time / control_period
    steps = int(round(steps_float))
    if steps < 1 or not np.isclose(steps, steps_float):
        raise ValueError('仿真时长必须是控制周期的整数倍')
    x = np.asarray(x0, dtype=float).copy()
    if x.shape != (6,) or not np.isfinite(x).all():
        raise ValueError('x0 必须为有限 6 维向量')
    states = np.zeros((steps + 1, 6))
    commands = np.zeros((steps, 3))
    forces = np.zeros((steps, 3))
    projection_times = np.zeros(steps)
    alphas = np.zeros(steps)
    solve_times = np.zeros((steps, 2))
    states[0] = x
    previous_projection = 0.0
    for i in range(steps):
        t_start = trajectory.project_time(x[:2], previous_time=previous_projection)
        previous_projection = t_start
        first, second, _, _, alpha = solve_two_pass_tracking(solver, trajectory, x, t_start)
        command = predicted_velocity_at(x, second.states, solver.config.dt, lookahead=0.01)
        # 简化的理想速度伺服：当前位置用旧速度推进，下一状态速度等于命令。
        next_x = x.copy()
        next_x[:3] += control_period * x[3:6]
        next_x[3:6] = command
        x = next_x
        states[i + 1] = x
        commands[i] = command
        forces[i] = second.controls[0]
        projection_times[i] = t_start
        alphas[i] = alpha[0]
        solve_times[i] = [first.solve_time, second.solve_time]
    return {
        'times': np.arange(steps + 1) * control_period,
        'states': states,
        'commands': commands,
        'forces': forces,
        'projection_times': projection_times,
        'alphas': alphas,
        'solve_times': solve_times,
    }


def plot_tracking_results(result: dict, trajectory: DynamicTrajectoryGenerator, save_path: str) -> None:
    """绘制仿真轨迹、预测速度命令和输入力。"""
    import matplotlib.pyplot as plt

    times = np.linspace(0.0, trajectory.duration, 500)
    path = np.array([trajectory.get_position(t) for t in times])
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    axes[0].plot(path[:, 0], path[:, 1], '--', label='Reference')
    axes[0].plot(result['states'][:, 0], result['states'][:, 1], label='Robot')
    axes[0].set(xlabel='x [m]', ylabel='y [m]', title='Trajectory')
    axes[0].axis('equal')
    axes[0].legend()
    for i, label in enumerate(('vx [m/s]', 'vy [m/s]', 'omega [rad/s]')):
        axes[1].plot(result['times'][:-1], result['commands'][:, i], label=label)
    axes[1].set(xlabel='Time [s]', title='10 ms predicted velocity')
    axes[1].legend()
    for i, label in enumerate(('Fx [N]', 'Fy [N]', 'Mz [N m]')):
        axes[2].plot(result['times'][:-1], result['forces'][:, i], label=label)
    axes[2].set(xlabel='Time [s]', title='First predicted input')
    axes[2].legend()
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def main() -> None:
    """使用明确标注的示例参数运行仿真；实际部署必须重新标定。"""
    # 以下质量、惯量、权重与限值都是演示假设，并非技术报告给出的实车参数。
    config = ReportMpcConfig(
        mass=30.0, inertia_z=2.0, dt=0.05, horizon_steps=40,
        q_diag=np.array([800.0, 800.0, 20.0, 100.0, 100.0, 5.0]),
        r_diag=np.array([0.001, 0.001, 0.5]),
        force_max=60.0, moment_max=2.0,
        state_lower=np.array([-np.inf, -np.inf, -np.inf, -2.0, -2.0, -1.5]),
        state_upper=np.array([np.inf, np.inf, np.inf, 2.0, 2.0, 1.5]),
    )
    simulation_time = 15.0
    trajectory = DynamicTrajectoryGenerator(scale=1.5, period=10.0, duration=17.0)
    solver = ReportMpcSolver(config)
    x0 = trajectory.get_full_state(0.0)
    x0[0] += 0.3
    x0[3:5] = 0.0
    print('[仿真] 40 步、50 ms 预测步长；qpOASES 双次求解，10 ms 速度前瞻')
    result = run_tracking_simulation(solver, trajectory, x0, simulation_time=simulation_time)
    plot_tracking_results(result, trajectory, 'mpc_report_figure8_qpoases.png')
    print(f"[完成] 两次 QP 平均求解时间: {1e3 * result['solve_times'].sum(axis=1).mean():.2f} ms/周期")
    print('[完成] 图像已保存: mpc_report_figure8_qpoases.png')


if __name__ == '__main__':
    main()
