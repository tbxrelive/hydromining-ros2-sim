"""阶段七规则决策层：补看/喷射/收尾的成本-收益循环，逐条输出可解释理由。

与 RL 共用同一动作语义：observe(view) / spray(u,v,duration) / finish。
适配层负责执行（替身环境或 ROS Action），本模块只读状态、输出动作与理由。
"""
import numpy as np

from .nbv import choose_view, uncertain_mask
from ..perception.assessment import circle_fractions


class RulePolicy:
    """证据不足的补看、缺口最大的喷射、达标后的收尾；每步记录决策依据。"""

    def __init__(self, config, observe_gain_min=0.05, standoff=0.7,
                 base_duration=6., max_duration=15., pressure=20.,
                 observe_budget=5):
        self.config_ = config
        self.observe_gain_min_ = observe_gain_min
        self.standoff_, self.pressure_ = standoff, pressure
        self.base_duration_, self.max_duration_ = base_duration, max_duration
        # 每轮补看上限：感知会话 maximum_views(8) 硬边界 − 3 个基线视点。
        self.observe_budget_ = observe_budget
        # 本轮已补看过的视点：重复观测不再产生信息，且会撞管线视点上限。
        self.observed_views_ = set()

    def reset_episode(self):
        """新回合开始：清空补看记录（截断结束的回合不会经由 finish 清空）。"""
        self.observed_views_.clear()

    def deficit_map(self, change, region):
        """区域内未达置信下界退缩量的单元及其缺口深度。

        目标是判决准则（delta - k·sigma >= required_depth），不是名义阈值；
        否则已名义达标但置信带跨界的单元会被漏喷，永远停在证据不足。
        """
        sigma = np.asarray(change['sigma'], float)
        bound = self.config_.required_depth + self.config_.confidence_k * np.where(
            np.isfinite(sigma), sigma, 0.)
        deficit = np.where(np.asarray(region, bool),
                           bound - np.asarray(change['delta'], float), np.nan)
        deficit[~np.asarray(change['valid'], bool)] = np.nan
        return deficit

    def spray_target(self, change, region, surface):
        """返回缺口最大单元的 (u,v) 与建议时长；无有效缺口返回 None。"""
        deficit = self.deficit_map(change, region)
        if not np.isfinite(deficit).any() or np.nanmax(deficit) <= 0:
            return None, 0.
        index = np.nanargmax(deficit)
        uv = surface.centers_[np.unravel_index(index, deficit.shape)][:2]
        # 时长与缺口深度成正比；未标定，属于工程缩放。
        duration = float(np.clip(self.base_duration_ * np.nanmax(deficit) /
                                 self.config_.required_depth, 2., self.max_duration_))
        return uv, duration

    def decide(self, state, assessment, change, surface, frame, view_poses,
               visibility, region_mask):
        """返回 {'type','detail','rationale','scores'}；assessment 为三分类报告。"""
        status = assessment['status']
        scores = {}
        if status == 'achieved':
            self.observed_views_.clear()
            return {'type': 'finish', 'rationale':
                    f'置信下界 {assessment["fraction_lower"]:.3f} >= 要求 '
                    f'{self.config_.required_fraction}，收尾', 'scores': scores}
        # 已补看过的视点从候选中剔除：同一轮重复观测不再产生新信息。
        remaining = dict(visibility)
        for seen in self.observed_views_:
            remaining[seen] = np.zeros_like(next(iter(remaining.values())))
        view, scores = choose_view(state, frame, view_poses, remaining,
                                   self.config_, region_mask)
        best_gain = scores[view]['gain'] if view else 0.
        observe_open = len(self.observed_views_) < self.observe_budget_
        if view is not None and observe_open and (assessment['coverage'] < 0.8 or
                                 status == 'insufficient_evidence' or
                                 best_gain > self.observe_gain_min_):
            self.observed_views_.add(view)
            return {'type': 'observe', 'view': view, 'scores': scores,
                    'rationale': f'覆盖率 {assessment["coverage"]:.2f} / 状态 {status}，'
                                 f'视点 {view} 预期 σ 改善 {best_gain:.3f} 最大'}
        uv, duration = self.spray_target(change, region_mask, surface)
        if uv is None:
            self.observed_views_.clear()
            if status == 'not_achieved' and assessment['fraction_lower'] < \
                    self.config_.required_fraction:
                return {'type': 'finish', 'scores': scores,
                        'rationale': '无有效缺口但未达标；按证据收尾报未达成'}
            budget_note = '' if observe_open else '，补看预算已用尽'
            return {'type': 'finish', 'scores': scores,
                    'rationale': f'无可喷射缺口{budget_note}，收尾'}
        deficit = float(np.nanmax(self.deficit_map(change, region_mask)))
        # 喷射后是新观测轮次，补看记录清空。
        self.observed_views_.clear()
        return {'type': 'spray', 'uv': uv, 'duration': duration, 'scores': scores,
                'rationale': f'缺口最大 {deficit * 1000:.1f} mm，位于 {np.round(uv, 3)}，'
                             f'喷射 {duration:.1f} s'}
