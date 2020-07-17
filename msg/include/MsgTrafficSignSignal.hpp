/** THIS IS AN AUTOMATICALLY GENERATED FILE.
 *  DO NOT MODIFY BY HAND!!
 *
 *  Generated by zcm-gen
 **/

#include <zcm/zcm_coretypes.h>

#ifndef __MsgTrafficSignSignal_hpp__
#define __MsgTrafficSignSignal_hpp__

#include <vector>
#include "TrafficSign.hpp"


class MsgTrafficSignSignal
{
    public:
        int64_t    timestamp;

        int8_t     num;

        std::vector< TrafficSign > traffic_sign_list;

    public:
        #if __cplusplus > 199711L /* if c++11 */
        static constexpr int8_t   IS_LITTLE_ENDIAN = 0;
        static constexpr int8_t   SpeedLimit5 = 0x01;
        static constexpr int8_t   SpeedLimit10 = 0x02;
        static constexpr int8_t   SpeedLimit15 = 0x03;
        static constexpr int8_t   SpeedLimit20 = 0x04;
        static constexpr int8_t   SpeedLimit25 = 0x05;
        static constexpr int8_t   SpeedLimit30 = 0x06;
        static constexpr int8_t   SpeedLimit35 = 0x07;
        static constexpr int8_t   SpeedLimit40 = 0x08;
        static constexpr int8_t   SpeedLimit50 = 0x09;
        static constexpr int8_t   SpeedLimit60 = 0x0A;
        static constexpr int8_t   SpeedLimit65 = 0x0B;
        static constexpr int8_t   SpeedLimit70 = 0x0C;
        static constexpr int8_t   SpeedLimit80 = 0x0D;
        static constexpr int8_t   SpeedLimit90 = 0x0E;
        static constexpr int8_t   SpeedLimit100 = 0x0F;
        static constexpr int8_t   SpeedLimit110 = 0x10;
        static constexpr int8_t   SpeedLimit120 = 0x11;
        static constexpr int8_t   Stop = 0x12;
        static constexpr int8_t   RigthGuide = 0x13;
        static constexpr int8_t   LeftGuide = 0x14;
        static constexpr int8_t   Tripod = 0x15;
        #else
        static const     int8_t   IS_LITTLE_ENDIAN = 0;
        static const     int8_t   SpeedLimit5 = 0x01;
        static const     int8_t   SpeedLimit10 = 0x02;
        static const     int8_t   SpeedLimit15 = 0x03;
        static const     int8_t   SpeedLimit20 = 0x04;
        static const     int8_t   SpeedLimit25 = 0x05;
        static const     int8_t   SpeedLimit30 = 0x06;
        static const     int8_t   SpeedLimit35 = 0x07;
        static const     int8_t   SpeedLimit40 = 0x08;
        static const     int8_t   SpeedLimit50 = 0x09;
        static const     int8_t   SpeedLimit60 = 0x0A;
        static const     int8_t   SpeedLimit65 = 0x0B;
        static const     int8_t   SpeedLimit70 = 0x0C;
        static const     int8_t   SpeedLimit80 = 0x0D;
        static const     int8_t   SpeedLimit90 = 0x0E;
        static const     int8_t   SpeedLimit100 = 0x0F;
        static const     int8_t   SpeedLimit110 = 0x10;
        static const     int8_t   SpeedLimit120 = 0x11;
        static const     int8_t   Stop = 0x12;
        static const     int8_t   RigthGuide = 0x13;
        static const     int8_t   LeftGuide = 0x14;
        static const     int8_t   Tripod = 0x15;
        #endif

    public:
        /**
         * Destructs a message properly if anything inherits from it
        */
        virtual ~MsgTrafficSignSignal() {}

        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void* buf, uint32_t offset, uint32_t maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline uint32_t getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to reqad while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void* buf, uint32_t offset, uint32_t maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "MsgTrafficSignSignal"
         */
        inline static const char* getTypeName();

        // ZCM support functions. Users should not call these
        inline int      _encodeNoHash(void* buf, uint32_t offset, uint32_t maxlen) const;
        inline uint32_t _getEncodedSizeNoHash() const;
        inline int      _decodeNoHash(const void* buf, uint32_t offset, uint32_t maxlen);
        inline static uint64_t _computeHash(const __zcm_hash_ptr* p);
};

int MsgTrafficSignSignal::encode(void* buf, uint32_t offset, uint32_t maxlen) const
{
    uint32_t pos = 0;
    int thislen;
    int64_t hash = (int64_t)getHash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int MsgTrafficSignSignal::decode(const void* buf, uint32_t offset, uint32_t maxlen)
{
    uint32_t pos = 0;
    int thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

uint32_t MsgTrafficSignSignal::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t MsgTrafficSignSignal::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* MsgTrafficSignSignal::getTypeName()
{
    return "MsgTrafficSignSignal";
}

int MsgTrafficSignSignal::_encodeNoHash(void* buf, uint32_t offset, uint32_t maxlen) const
{
    uint32_t pos = 0;
    int thislen;

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->timestamp, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->num, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    for (int a0 = 0; a0 < this->num; ++a0) {
        thislen = this->traffic_sign_list[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(thislen < 0) return thislen; else pos += thislen;
    }

    return pos;
}

int MsgTrafficSignSignal::_decodeNoHash(const void* buf, uint32_t offset, uint32_t maxlen)
{
    uint32_t pos = 0;
    int thislen;

    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->timestamp, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->num, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    this->traffic_sign_list.resize(this->num);
    for (int a0 = 0; a0 < this->num; ++a0) {
        thislen = this->traffic_sign_list[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(thislen < 0) return thislen; else pos += thislen;
    }

    return pos;
}

uint32_t MsgTrafficSignSignal::_getEncodedSizeNoHash() const
{
    uint32_t enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int8_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->num; ++a0) {
        enc_size += this->traffic_sign_list[a0]._getEncodedSizeNoHash();
    }
    return enc_size;
}

uint64_t MsgTrafficSignSignal::_computeHash(const __zcm_hash_ptr* p)
{
    const __zcm_hash_ptr* fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == MsgTrafficSignSignal::getHash)
            return 0;
    const __zcm_hash_ptr cp = { p, (void*)MsgTrafficSignSignal::getHash };

    uint64_t hash = (uint64_t)0xdecae4e88d4a1859LL +
         TrafficSign::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

#endif
