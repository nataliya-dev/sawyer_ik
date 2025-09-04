import numpy as np

from pydrake.all import (
    InverseKinematics,
    Solve,
    RotationMatrix,
    Quaternion,
)


class KinematicsSolver:
    def __init__(self):
        print("Hello, I'm a kinematics solver!")

    def normalize(self, x):
        return x / np.linalg.norm(x)

    def solve_ik(self, plant, pos_W, q_W, ee_link_name, q_seed=None, tol=1e-3):

        E = plant.GetBodyByName(ee_link_name).body_frame()
        W = plant.world_frame()

        ctx = plant.CreateDefaultContext()
        ik = InverseKinematics(plant, ctx)
        prog = ik.prog()

        q = ik.q()
        if q_seed is None and self.x0 is not None:
            q_seed = self.x0
        else:
            # need to have nonzero values here
            q_seed = np.ones(plant.num_positions()) * 0.1

        prog.SetInitialGuess(q, q_seed)

        # position eq constraint
        p_AQ_lower = np.array(pos_W) - tol
        p_AQ_upper = np.array(pos_W) + tol
        print(f"{pos_W=}")
        print(f"{p_AQ_lower=}")
        print(f"{p_AQ_upper=}")

        ik.AddPositionConstraint(
            frameA=W,
            p_BQ=np.zeros(3),
            frameB=E,
            p_AQ_lower=p_AQ_lower,
            p_AQ_upper=p_AQ_upper,
        )

        if q_W is not None:
            I = RotationMatrix()
            desired = RotationMatrix(quaternion=Quaternion(wxyz=self.normalize(q_W)))
            ik.AddOrientationConstraint(
                R_AbarA=I, frameAbar=W, R_BbarB=desired, frameBbar=E, theta_bound=tol
            )

        # bounds inside joint limits
        print(f"{plant.GetPositionLowerLimits()=}")
        print(f"{plant.GetPositionUpperLimits()=}")
        prog.AddBoundingBoxConstraint(
            plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits(), q
        )

        result = Solve(prog)
        if not result.is_success():
            return result.is_success(), q_seed.copy()
        return result.is_success(), result.GetSolution(q)


class SawyerKinematicsSolver(KinematicsSolver):
    def __init__(self):
        super().__init__()
        print("Hello, I'm a sawyer kinematics solver!")
        self.x0 = [0.0, 0.0, -1.5708, 1.5708, 0.0, -1.5708, 0.0]
        self.ee_link_name = "right_hand"
