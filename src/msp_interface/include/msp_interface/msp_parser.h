#ifndef MSP_INTERFACE_MSP_PARSER_H
#define MSP_INTERFACE_MSP_PARSER_H

#include <cstdint>
#include <vector>

namespace msp_interface
{

/**
 * @brief MSP V2 协议解析器（状态机实现）
 * 
 * 逐字节解析 MSP V2 帧，提取有效负载并进行 CRC 校验。
 * 使用示例：
 *   MspV2Parser parser;
 *   std::vector<uint8_t> payload;
 *   for (uint8_t byte : serial_data) {
 *       if (parser.parseByte(byte, payload)) {
 *           // 成功解析出一个完整包，payload 中包含有效数据
 *           handlePacket(payload);
 *       }
 *   }
 */
class MspV2Parser
{
public:
    /**
     * @brief 解析器内部状态枚举
     */
    enum State
    {
        WAIT_HEADER1,   ///< 等待 '$'
        WAIT_HEADER2,   ///< 等待 'X'
        WAIT_FLAG,      ///< 等待标志字节
        WAIT_CMD1,      ///< 等待命令低字节
        WAIT_CMD2,      ///< 等待命令高字节
        WAIT_SIZE1,     ///< 等待长度低字节
        WAIT_SIZE2,     ///< 等待长度高字节
        WAIT_PAYLOAD,   ///< 等待负载数据
        WAIT_CRC1,      ///< 等待 CRC 低字节
        WAIT_CRC2       ///< 等待 CRC 高字节
    };

    /**
     * @brief 构造函数，初始化状态机
     */
    MspV2Parser();

    /**
     * @brief 重置解析器状态（丢弃当前正在解析的包）
     */
    void reset();

    /**
     * @brief 逐字节解析
     * @param byte       输入字节
     * @param out_packet 当返回 true 时，此向量包含解析出的有效负载（不含帧头、命令、长度、CRC）
     * @return true 表示已解析出一个完整且 CRC 正确的数据包，有效负载可通过 out_packet 获取；
     *         false 表示尚未完成一个包或包错误。
     */
    bool parseByte(uint8_t byte, std::vector<uint8_t>& out_packet);

private:
    State state_;                ///< 当前解析状态
    uint8_t flag_;                ///< 标志字节
    uint16_t cmd_;                ///< 命令 ID
    uint16_t payload_size_;       ///< 负载长度（从帧中解析）
    std::vector<uint8_t> payload_; ///< 临时存储负载数据
    uint16_t crc_calc_;            ///< 计算中的 CRC 值
    uint16_t crc_received_;        ///< 接收到的 CRC 值
};

} // namespace msp_interface

#endif // MSP_INTERFACE_MSP_PARSER_H