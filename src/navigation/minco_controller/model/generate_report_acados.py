#!/usr/bin/env python3
"""Generate the acados solver for omnidirectional_report_tracking.py.

State is [x, y, yaw, vx, vy, omega], input is [Fx, Fy, Mz], and the
discrete dynamics use the report's forward Euler discretization.  The
parameters are [dt, mass, inertia_z].  Stages 1..N track x_1..x_N;
stage 0 has no state cost and all N stages penalize the input.

Requires CasADi, acados_template and an acados installation. Set
ACADOS_SOURCE_DIR to the acados installation before running this script.
"""

from pathlib import Path

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver


def make_ocp():
    model = AcadosModel()
    model.name = 'report_mpc'
    model.x = ca.SX.sym('x', 6)
    model.u = ca.SX.sym('u', 3)
    model.p = ca.SX.sym('p', 3)
    dt, mass, inertia = model.p[0], model.p[1], model.p[2]
    model.disc_dyn_expr = model.x + ca.vertcat(
        dt * model.x[3], dt * model.x[4], dt * model.x[5],
        dt * model.u[0] / mass, dt * model.u[1] / mass,
        dt * model.u[2] / inertia,
    )

    ocp = AcadosOcp()
    ocp.name = 'report_mpc'
    ocp.model = model
    ocp.parameter_values = np.array([0.05, 30.0, 2.0])
    ocp.solver_options.N_horizon = 40
    # Unit numerical time steps keep the least-squares cost unscaled. The
    # physical sampling period is supplied through the dynamics parameter.
    ocp.solver_options.tf = 40.0
    ocp.solver_options.integrator_type = 'DISCRETE'
    ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'

    q = np.diag([800., 800., 20., 100., 100., 5.])
    r = np.diag([0.001, 0.001, 0.5])
    w = np.zeros((9, 9))
    w[:6, :6] = q
    w[6:, 6:] = r
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.W = w
    ocp.cost.Vx = np.vstack((np.eye(6), np.zeros((3, 6))))
    ocp.cost.Vu = np.vstack((np.zeros((6, 3)), np.eye(3)))
    ocp.cost.yref = np.zeros(9)
    ocp.cost.cost_type_0 = 'LINEAR_LS'
    ocp.cost.W_0 = w.copy()
    ocp.cost.W_0[:6, :6] = 0.0
    ocp.cost.Vx_0 = ocp.cost.Vx.copy()
    ocp.cost.Vu_0 = ocp.cost.Vu.copy()
    ocp.cost.yref_0 = np.zeros(9)
    ocp.cost.cost_type_e = 'LINEAR_LS'
    ocp.cost.W_e = q
    ocp.cost.Vx_e = np.eye(6)
    ocp.cost.yref_e = np.zeros(6)

    ocp.constraints.x0 = np.zeros(6)
    ocp.constraints.idxbu = np.arange(3)
    ocp.constraints.lbu = np.array([-60., -60., -10.])
    ocp.constraints.ubu = -ocp.constraints.lbu
    ocp.constraints.idxbx = np.array([3, 4, 5])
    ocp.constraints.lbx = np.array([-3., -3., -5.])
    ocp.constraints.ubx = -ocp.constraints.lbx
    ocp.constraints.idxbx_e = ocp.constraints.idxbx.copy()
    ocp.constraints.lbx_e = ocp.constraints.lbx.copy()
    ocp.constraints.ubx_e = ocp.constraints.ubx.copy()
    return ocp


def main():
    output_dir = Path(__file__).resolve().parent / 'report_codegen'
    ocp = make_ocp()
    ocp.code_gen_options.code_export_directory = str(output_dir)
    ocp.code_gen_options.json_file = str(output_dir / 'acados_ocp.json')
    AcadosOcpSolver.generate(ocp)
    for name in ('acados_ocp.json', 'Makefile', 'acados_solver.pxd', 'main_report_mpc.c'):
        (output_dir / name).unlink(missing_ok=True)


if __name__ == '__main__':
    main()
