/**
 * @file crc.hpp
 * @brief CRC8/CRC16 校验算法接口声明。
 */

#ifndef TOOLS__CRC_HPP
#define TOOLS__CRC_HPP

#include <cstdint>

namespace tools {
/**
 * @brief 计算 CRC8 校验值。
 * @param data 待校验数据首地址。
 * @param len 不包含 CRC8 字节的数据长度。
 * @return CRC8 校验值。
 * @note len不包括crc8。
 */
uint8_t get_crc8(const uint8_t* data, uint16_t len);

/**
 * @brief 校验数据末尾携带的 CRC8。
 * @param data 待校验数据首地址。
 * @param len 包含 CRC8 字节的数据长度。
 * @return 校验通过返回 true，否则返回 false。
 * @note len包括crc8。
 */
bool check_crc8(const uint8_t* data, uint16_t len);

/**
 * @brief 计算 CRC16 校验值。
 * @param data 待校验数据首地址。
 * @param len 不包含 CRC16 字节的数据长度。
 * @return CRC16 校验值。
 * @note len不包括crc16。
 */
uint16_t get_crc16(const uint8_t* data, uint32_t len);

/**
 * @brief 校验数据末尾携带的 CRC16。
 * @param data 待校验数据首地址。
 * @param len 包含 CRC16 字节的数据长度。
 * @return 校验通过返回 true，否则返回 false。
 * @note len包括crc16。
 */
bool check_crc16(const uint8_t* data, uint32_t len);

} // namespace tools

#endif // TOOLS__CRC_HPP
