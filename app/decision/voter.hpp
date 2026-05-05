/**
 * @file voter.hpp
 * @brief 装甲板识别投票计数器接口，按颜色、编号和大小统计识别次数
 */

#ifndef AUTO_AIM__VOTER_HPP
#define AUTO_AIM__VOTER_HPP

#include <vector>

#include "../auto_aim/armor.hpp"

namespace auto_aim {

/**
 * @brief 统计重复出现的装甲板分类结果，让策略可以基于更稳定的身份判断
 *
 * 使用紧凑的一维数组，是因为颜色、编号和大小类型的组合数量固定且很小，直接索引比映射结构更轻量
 */
class Voter {
  public:
    /**
     * @brief 为所有颜色、编号和大小组合初始化计数器，避免首次访问时动态创建带来的不确定性
     */
    Voter();
    /**
     * @brief 给一次装甲板身份识别结果增加投票，用于后续稳定性判断
     * @param color 装甲板颜色，用作投票桶的第一维索引
     * @param name 装甲板编号，用作投票桶的第二维索引
     * @param type 装甲板大小类型，用作投票桶的第三维索引
     */
    void vote(const Color color, const ArmorName name, const ArmorType type);
    /**
     * @brief 返回某个装甲板身份的累计票数，便于选择最可信分类结果
     * @param color 装甲板颜色，用作投票桶的第一维索引
     * @param name 装甲板编号，用作投票桶的第二维索引
     * @param type 装甲板大小类型，用作投票桶的第三维索引
     * @return 当前组合累计出现次数，用于判断多帧识别结果是否稳定
     */
    std::size_t count(const Color color, const ArmorName name, const ArmorType type);

  private:
    std::vector<std::size_t> count_;
    std::size_t index(const Color color, const ArmorName name, const ArmorType type) const;
};
} // namespace auto_aim

#endif