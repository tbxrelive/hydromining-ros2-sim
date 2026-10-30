"""固定煤壁坐标系下的有界 2.5D 鲁棒融合，不填补未观测单元格。"""
import numpy as np


class HeightMap:
    """每帧逐格聚合后融合，独立保留系统误差下限与观测次数。"""

    def __init__(self, frame, extent=0.5, resolution=0.01, systematic_sigma=0.0015,
                 maximum_height=0.25, huber_k=2.5, minimum_incidence=0.15):
        self.frame_, self.extent_, self.resolution_ = frame, extent, resolution
        self.centers_ = frame.grid(extent, resolution)
        self.shape_ = self.centers_.shape[:2]
        if systematic_sigma <= 0 or maximum_height <= 0 or huber_k <= 0:
            raise ValueError('invalid map uncertainty or bounds')
        self.systematic_sigma_ = systematic_sigma
        self.maximum_height_, self.huber_k_ = maximum_height, huber_k
        self.minimum_incidence_ = minimum_incidence
        self.mean_ = np.full(self.shape_, np.nan)
        self.precision_ = np.zeros(self.shape_)
        self.count_ = np.zeros(self.shape_, dtype=np.int32)
        self.points_ = np.zeros(self.shape_, dtype=np.int32)
        self.ambiguous_ = np.zeros(self.shape_, dtype=bool)
        self.scatter_ = np.zeros(self.shape_)
        self.estimate_mean_ = np.zeros(self.shape_)
        self.model_variance_ = np.zeros(self.shape_)
        self.slope_sum_ = np.zeros((*self.shape_, 2))
        self.slope_weight_ = np.zeros(self.shape_)

    def indices(self, local):
        """返回行列和界内掩码；不裁剪越界点到边界格。"""
        local = np.asarray(local, float)
        valid = np.isfinite(local).all(axis=-1)
        safe = np.where(valid[..., None], local, 0.0)
        col = np.floor((safe[..., 0] + self.extent_ / 2) / self.resolution_).astype(int)
        row = np.floor((safe[..., 1] + self.extent_ / 2) / self.resolution_).astype(int)
        valid &= ((row >= 0) & (row < self.shape_[0]) &
                  (col >= 0) & (col < self.shape_[1]) &
                  (np.abs(safe[..., 2]) < self.maximum_height_))
        return row, col, valid

    def reference_at(self, world):
        """查已有可靠高度；缺失格保持 NaN，不插值出新的几何证据。"""
        local = self.frame_.to_local(world)
        row, col, valid = self.indices(local)
        result = np.full(row.shape, np.nan)
        result[valid] = self.mean_[row[valid], col[valid]]
        return result

    def integrate(self, world, directions, range_sigma):
        """按射线法向投影传播方差，逐格用局部平面中心高度稳健融合。"""
        world = np.asarray(world, float).reshape(-1, 3)
        directions = np.asarray(directions, float).reshape(-1, 3)
        sigma = np.asarray(range_sigma, float).reshape(-1)
        if len(world) != len(directions) or len(world) != len(sigma):
            raise ValueError('measurement arrays must have equal lengths')
        local = self.frame_.to_local(world)
        row, col, valid = self.indices(local)
        incidence = np.abs(directions @ self.frame_.axes[:, 2])
        valid &= np.isfinite(sigma) & (sigma >= 0) & (incidence >= self.minimum_incidence_)
        variance = np.maximum((sigma * incidence) ** 2, 1e-8)
        flat = row[valid] * self.shape_[1] + col[valid]
        order = np.argsort(flat)
        flat, selected, var = flat[order], local[valid][order], variance[valid][order]
        unique, starts = np.unique(flat, return_index=True)
        for key, start, end in zip(unique, starts, np.r_[starts[1:], len(flat)]):
            r, c = divmod(int(key), self.shape_[1])
            points, variances = selected[start:end], var[start:end]
            heights = points[:, 2]
            median = np.median(heights)
            mad = 1.4826 * np.median(np.abs(heights - median))
            keep = np.abs(heights - median) <= max(3.5 * mad, 0.005)
            points, variances = points[keep], variances[keep]
            if not len(points):
                continue
            heights = points[:, 2]
            weights = 1 / variances
            design = np.column_stack((points[:, :2] - self.centers_[r, c, :2], np.ones(len(points))))
            normal_matrix = design.T @ (weights[:, None] * design)
            if len(points) >= 6 and np.linalg.matrix_rank(normal_matrix, tol=1e-5) == 3:
                covariance = np.linalg.inv(normal_matrix)
                parameters = covariance @ (design.T @ (weights * heights))
                estimate = parameters[2]
                residual = heights - design @ parameters
                scale = max(1., float(np.sum(weights * residual ** 2) / max(1, len(points) - 3)))
                estimate_variance = covariance[2, 2] * scale
                slope = parameters[:2]
                model_variance = float(np.mean(residual ** 2))
            else:
                estimate = np.average(heights, weights=weights)
                residual = heights - estimate
                estimate_variance = max(1 / weights.sum(),
                                        float(np.var(residual) / max(1, len(points))))
                slope = None
                model_variance = float(np.var(residual))
            # 多模态只能标为不支持；不能强行平均成一个不存在的表面。
            ordered = np.sort(heights)
            if len(ordered) >= 8:
                lo, hi = len(ordered) // 4, 3 * len(ordered) // 4
                self.ambiguous_[r, c] |= bool(np.any(np.diff(ordered)[lo:hi] > 0.04))
            precision = 1 / max(estimate_variance, 1e-8)
            observation_count = self.count_[r, c] + 1
            difference = estimate - self.estimate_mean_[r, c]
            self.estimate_mean_[r, c] += difference / observation_count
            self.scatter_[r, c] += difference * (estimate - self.estimate_mean_[r, c])
            self.model_variance_[r, c] = max(self.model_variance_[r, c], model_variance)
            if slope is not None:
                self.slope_sum_[r, c] += slope * precision
                self.slope_weight_[r, c] += precision
            if self.precision_[r, c] > 0:
                residual = abs(estimate - self.mean_[r, c])
                bound = self.huber_k_ * np.sqrt(
                    1 / self.precision_[r, c] + estimate_variance + self.systematic_sigma_ ** 2)
                precision *= min(1., bound / max(residual, 1e-12))
                estimate = (self.mean_[r, c] * self.precision_[r, c] +
                            estimate * precision) / (self.precision_[r, c] + precision)
            self.mean_[r, c] = estimate
            self.precision_[r, c] += precision
            self.count_[r, c] += 1
            self.points_[r, c] += len(points)

    def state(self, minimum_observations=2, maximum_sigma=0.01):
        """覆盖状态 0 未观测、1 不足、2 可靠；系统误差不随重复平均消失。"""
        variance = np.full(self.shape_, np.nan)
        np.divide(1., self.precision_, out=variance, where=self.precision_ > 0)
        disagreement = np.zeros(self.shape_)
        np.divide(self.scatter_, self.count_ - 1, out=disagreement, where=self.count_ > 1)
        sigma = np.sqrt(variance + self.systematic_sigma_ ** 2 +
                        self.model_variance_ + disagreement)
        coverage = np.where(self.count_ > 0, 1, 0).astype(np.int8)
        reliable = ((self.count_ >= minimum_observations) & (sigma <= maximum_sigma) &
                    ~self.ambiguous_)
        coverage[reliable] = 2
        normals = self.normals(reliable)
        return {'height': self.mean_.copy(), 'sigma': sigma, 'count': self.count_.copy(),
                'points': self.points_.copy(), 'coverage': coverage, 'normal': normals,
                'ambiguous': self.ambiguous_.copy()}

    def normals(self, reliable):
        """用可靠相邻格局部平面估计法向；邻域不足不输出伪法向。"""
        normals = np.full((*self.shape_, 3), np.nan)
        for r, c in np.argwhere(reliable):
            if self.slope_weight_[r, c] > 0:
                slope = self.slope_sum_[r, c] / self.slope_weight_[r, c]
                local_normal = np.array([-slope[0], -slope[1], 1.])
                normals[r, c] = self.frame_.axes @ (local_normal / np.linalg.norm(local_normal))
                continue
            rr = slice(max(0, r - 1), min(self.shape_[0], r + 2))
            cc = slice(max(0, c - 1), min(self.shape_[1], c + 2))
            keep = reliable[rr, cc]
            if keep.sum() < 5:
                continue
            uv = self.centers_[rr, cc, :2][keep]
            design = np.column_stack((uv - self.centers_[r, c, :2], np.ones(len(uv))))
            if np.linalg.matrix_rank(design) < 3:
                continue
            slope = np.linalg.lstsq(design, self.mean_[rr, cc][keep], rcond=None)[0]
            local_normal = np.array([-slope[0], -slope[1], 1.])
            normals[r, c] = self.frame_.axes @ (local_normal / np.linalg.norm(local_normal))
        return normals
