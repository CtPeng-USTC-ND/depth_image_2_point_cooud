# depth_image_2_point_cooud
def depth_to_pcd(depth, intrinsics, depth_scale=1.0, max_depth_thresh=None):
    z = depth * depth_scale
    row = np.linspace(0, depth.shape[0] - 1, depth.shape[0])
    col = np.linspace(0, depth.shape[1] - 1, depth.shape[1])
    u, v = np.meshgrid(col, row)
    x = (u - intrinsics['cx']) * z / intrinsics['fx']
    y = (v - intrinsics['cy']) * z / intrinsics['fy']
    xyz = np.vstack(
        (np.reshape(x, -1), np.reshape(y, -1), np.reshape(z, -1))) \
        .transpose(1, 0)
    if max_depth_thresh is not None:
        xyz = xyz[
            np.where((xyz[:, 2] > 0) & (xyz[:, 2] < max_depth_thresh))[0], :]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    return pcd
    
src_part_depth_path = f'{root_dir}/1652840815981.png'
im_intrinsics = {'fx': 898.54, 'fy': 898.731, 'cx': 654.874, 'cy': 371.738}
depth_scale = 0.00025 # in meter
max_depth_thresh = 3 # in meter
src_part_pcd = depth_to_pcd(src_part_depth, im_intrinsics, depth_scale, max_depth_thresh)


ref_part_mesh = o3d.io.read_triangle_mesh(ref_part_model_path)
ref_part_pcd = ref_part_mesh.sample_points_uniformly(num_sample_per_part)
pts = np.asarray(ref_part_pcd.points)
pts *= 0.001 # from millimeter (mm) to meter (m)
