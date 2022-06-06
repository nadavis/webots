from ikpy.chain import Chain

class Kinematics():
    def __init__(self, filename):
        self.IKPY_MAX_ITERATIONS = 8

        self.armChain = Chain.from_urdf_file(filename)
        self.armChain.active_links_mask[0] = False
        self.armChain.active_links_mask[6] = False
        self.armChain.active_links_mask[7] = False

    def run_inverse_kinematics_positions(self, coordinates, initial_coordinates):
        self.armChain.active_links_mask[4] = False
        self.armChain.active_links_mask[5] = False
        ikResults = self.armChain.inverse_kinematics(
            target_position=coordinates,
            max_iter=self.IKPY_MAX_ITERATIONS,
            initial_position=initial_coordinates)

        self.armChain.active_links_mask[4] = True
        self.armChain.active_links_mask[5] = True

        return ikResults

    def run_inverse_kinematics_positions_orientation(self, coordinates, target_orientation, mode):
        ikResults = self.armChain.inverse_kinematics(
            target_position=coordinates,
            target_orientation=target_orientation,
            orientation_mode=mode)

        return ikResults

    def run_forward_kinematics(self, positions):
        return self.armChain.forward_kinematics(positions)
