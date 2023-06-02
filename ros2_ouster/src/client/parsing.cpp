/**
* Copyright (c) 2018, Ouster, Inc.
* All rights reserved.
*/

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "ros2_ouster/client/types.h"

//FIXME(debug): remove this
#include <iostream>

namespace ouster {
namespace sensor {

namespace impl {

constexpr int imu_packet_size = 48;
constexpr int64_t encoder_ticks_per_rev = 90112;

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

struct FieldInfo {
 ChanFieldType ty_tag;
 size_t offset;
 uint64_t mask;
 int shift;
};

struct ProfileEntry {
 const std::pair<ChanField, FieldInfo>* fields;
 size_t n_fields;
 size_t chan_data_size;
};

static const Table<ChanField, FieldInfo, 8> legacy_field_info{{
       {ChanField::RANGE, {UINT32, 0, 0x000fffff, 0}},
       {ChanField::FLAGS, {UINT8, 3, 0, 4}},
       {ChanField::REFLECTIVITY, {UINT16, 4, 0, 0}},
       {ChanField::SIGNAL, {UINT16, 6, 0, 0}},
       {ChanField::NEAR_IR, {UINT16, 8, 0, 0}},
       {ChanField::RAW32_WORD1, {UINT32, 0, 0, 0}},
       {ChanField::RAW32_WORD2, {UINT32, 4, 0, 0}},
       {ChanField::RAW32_WORD3, {UINT32, 8, 0, 0}},
}};

static const Table<ChanField, FieldInfo, 5> lb_field_info{{
       {ChanField::RANGE, {UINT16, 0, 0x7fff, -3}},
       {ChanField::FLAGS, {UINT8, 1, 0b10000000, 7}},
       {ChanField::REFLECTIVITY, {UINT8, 2, 0, 0}},
       {ChanField::NEAR_IR, {UINT8, 3, 0, -4}},
       {ChanField::RAW32_WORD1, {UINT32, 0, 0, 0}},
}};

static const Table<ChanField, FieldInfo, 13> dual_field_info{{
       {ChanField::RANGE, {UINT32, 0, 0x0007ffff, 0}},
       {ChanField::FLAGS, {UINT8, 2, 0b11111000, 3}},
       {ChanField::REFLECTIVITY, {UINT8, 3, 0, 0}},
       {ChanField::RANGE2, {UINT32, 4, 0x0007ffff, 0}},
       {ChanField::FLAGS2, {UINT8, 6, 0b11111000, 3}},
       {ChanField::REFLECTIVITY2, {UINT8, 7, 0, 0}},
       {ChanField::SIGNAL, {UINT16, 8, 0, 0}},
       {ChanField::SIGNAL2, {UINT16, 10, 0, 0}},
       {ChanField::NEAR_IR, {UINT16, 12, 0, 0}},
       {ChanField::RAW32_WORD1, {UINT32, 0, 0, 0}},
       {ChanField::RAW32_WORD2, {UINT32, 4, 0, 0}},
       {ChanField::RAW32_WORD3, {UINT32, 8, 0, 0}},
       {ChanField::RAW32_WORD4, {UINT32, 12, 0, 0}},
}};

static const Table<ChanField, FieldInfo, 8> single_field_info{{
       {ChanField::RANGE, {UINT32, 0, 0x0007ffff, 0}},
       {ChanField::FLAGS, {UINT8, 2, 0b11111000, 3}},
       {ChanField::REFLECTIVITY, {UINT8, 4, 0, 0}},
       {ChanField::SIGNAL, {UINT16, 6, 0, 0}},
       {ChanField::NEAR_IR, {UINT16, 8, 0, 0}},
       {ChanField::RAW32_WORD1, {UINT32, 0, 0, 0}},
       {ChanField::RAW32_WORD2, {UINT32, 4, 0, 0}},
       {ChanField::RAW32_WORD3, {UINT32, 8, 0, 0}},
}};

static const Table<ChanField, FieldInfo, 14> five_word_pixel_info{{
       {ChanField::RANGE, {UINT32, 0, 0x0007ffff, 0}},
       {ChanField::FLAGS, {UINT8, 2, 0b11111000, 3}},
       {ChanField::REFLECTIVITY, {UINT8, 3, 0, 0}},
       {ChanField::RANGE2, {UINT32, 4, 0x0007ffff, 0}},
       {ChanField::FLAGS2, {UINT8, 6, 0b11111000, 3}},
       {ChanField::REFLECTIVITY2, {UINT8, 7, 0, 0}},
       {ChanField::SIGNAL, {UINT16, 8, 0, 0}},
       {ChanField::SIGNAL2, {UINT16, 10, 0, 0}},
       {ChanField::NEAR_IR, {UINT16, 12, 0, 0}},
       {ChanField::RAW32_WORD1, {UINT32, 0, 0, 0}},
       {ChanField::RAW32_WORD2, {UINT32, 4, 0, 0}},
       {ChanField::RAW32_WORD3, {UINT32, 8, 0, 0}},
       {ChanField::RAW32_WORD4, {UINT32, 12, 0, 0}},
       {ChanField::RAW32_WORD5, {UINT32, 16, 0, 0}},
}};

Table<UDPProfileLidar, ProfileEntry, 32> profiles{{
       {UDPProfileLidar::PROFILE_LIDAR_LEGACY,
        {legacy_field_info.data(), legacy_field_info.size(), 12}},
       {UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL,
        {dual_field_info.data(), dual_field_info.size(), 16}},
       {UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16,
        {single_field_info.data(), single_field_info.size(), 12}},
       {UDPProfileLidar::PROFILE_RNG15_RFL8_NIR8,
        {lb_field_info.data(), lb_field_info.size(), 4}},
       {UDPProfileLidar::PROFILE_FIVE_WORD_PIXEL,
        {five_word_pixel_info.data(), five_word_pixel_info.size(), 20}},
}};

static const ProfileEntry& lookup_profile_entry(UDPProfileLidar profile) {
 auto end = profiles.end();
 auto it =
         std::find_if(impl::profiles.begin(), end,
                      [profile](const auto& kv) { return kv.first == profile; });

 if (it == end || it->first == 0)
   throw std::invalid_argument("Unknown lidar udp profile");

 return it->second;
}

}  // namespace impl

struct packet_format::Impl {
 size_t packet_header_size;
 size_t col_header_size;
 size_t channel_data_size;
 size_t col_footer_size;
 size_t packet_footer_size;

 size_t col_size;
 size_t lidar_packet_size;

 size_t timestamp_offset;
 size_t measurement_id_offset;
 size_t status_offset;

 std::map<ChanField, impl::FieldInfo> fields;

 Impl(UDPProfileLidar profile, int pixels_per_column,
      int columns_per_packet) {
   bool legacy = (profile == UDPProfileLidar::PROFILE_LIDAR_LEGACY);

   const auto& entry = impl::lookup_profile_entry(profile);

   packet_header_size = legacy ? 0 : 32;
   col_header_size = legacy ? 16 : 12;
   channel_data_size = entry.chan_data_size;
   col_footer_size = legacy ? 4 : 0;
   packet_footer_size = legacy ? 0 : 32;

   col_size = col_header_size + pixels_per_column * channel_data_size +
              col_footer_size;
   lidar_packet_size = packet_header_size + columns_per_packet * col_size +
                       packet_footer_size;

   fields = {entry.fields, entry.fields + entry.n_fields};

   timestamp_offset = 0;
   measurement_id_offset = 8;
   status_offset = legacy ? col_size - col_footer_size : 10;
 }
};

packet_format::packet_format(const sensor_info& info)
   : impl_{std::make_shared<Impl>(info.format.udp_profile_lidar,
                                  info.format.pixels_per_column,
                                  info.format.columns_per_packet)},
     udp_profile_lidar{info.format.udp_profile_lidar},
     lidar_packet_size{impl_->lidar_packet_size},
     imu_packet_size{impl::imu_packet_size},
     columns_per_packet(info.format.columns_per_packet),
     pixels_per_column(info.format.pixels_per_column),
     packet_header_size{impl_->packet_header_size},
     col_header_size{impl_->col_header_size},
     col_footer_size{impl_->col_footer_size},
     col_size{impl_->col_size},
     packet_footer_size{impl_->packet_footer_size} {
 for (const auto& kv : impl_->fields) {
   field_types_.emplace_back(kv.first, kv.second.ty_tag);
 }
}

template <typename SRC, typename DST>
static void col_field_impl(const uint8_t* col_buf, DST* dst, size_t offset,
                          uint64_t mask, int shift, int pixels_per_column,
                          int dst_stride, size_t channel_data_size,
                          size_t col_header_size) {
 if (sizeof(DST) < sizeof(SRC))
   throw std::invalid_argument("Dest type too small for specified field");

 for (int px = 0; px < pixels_per_column; px++) {
   auto px_src =
           col_buf + col_header_size + offset + (px * channel_data_size);
   DST* px_dst = dst + px * dst_stride;
   *px_dst = 0;
   std::memcpy(px_dst, px_src, sizeof(SRC));
   if (mask) *px_dst &= mask;
   if (shift > 0) *px_dst >>= shift;
   if (shift < 0) *px_dst <<= std::abs(shift);
 }
}

template <typename T,
        typename std::enable_if<std::is_unsigned<T>::value, T>::type>
void packet_format::col_field(const uint8_t* col_buf, ChanField i, T* dst,
                             int dst_stride) const {
 //FIXME(debug): remove this
// std::cout << "in enable_if" << std::endl;
 const auto& f = impl_->fields.at(i);

 switch (f.ty_tag) {
   case UINT8:
     col_field_impl<uint8_t, T>(
             col_buf, dst, f.offset, f.mask, f.shift, pixels_per_column,
             dst_stride, impl_->channel_data_size, impl_->col_header_size);
     break;
   case UINT16:
     col_field_impl<uint16_t, T>(
             col_buf, dst, f.offset, f.mask, f.shift, pixels_per_column,
             dst_stride, impl_->channel_data_size, impl_->col_header_size);
     break;
   case UINT32:
     col_field_impl<uint32_t, T>(
             col_buf, dst, f.offset, f.mask, f.shift, pixels_per_column,
             dst_stride, impl_->channel_data_size, impl_->col_header_size);
     break;
   case UINT64:
     col_field_impl<uint64_t, T>(
             col_buf, dst, f.offset, f.mask, f.shift, pixels_per_column,
             dst_stride, impl_->channel_data_size, impl_->col_header_size);
     break;
   default:
     throw std::invalid_argument("Invalid field for packet format");
 }
}

// explicitly instantiate for each field type
template void packet_format::col_field(const uint8_t*, ChanField, uint8_t*,
                                      int) const;
template void packet_format::col_field(const uint8_t*, ChanField, uint16_t*,
                                      int) const;
template void packet_format::col_field(const uint8_t*, ChanField, uint32_t*,
                                      int) const;
template void packet_format::col_field(const uint8_t*, ChanField, uint64_t*,
                                      int) const;

ChanFieldType packet_format::field_type(ChanField f) const {
 //FIXME(debug): remove this
// std::cout << "in filed_type" << std::endl;
 return impl_->fields.count(f) ? impl_->fields.at(f).ty_tag
                               : ChanFieldType::VOID;
}

packet_format::FieldIter packet_format::begin() const {
 return field_types_.cbegin();
}

packet_format::FieldIter packet_format::end() const {
 return field_types_.cend();
}

/* Packet headers */

uint16_t packet_format::packet_type(const uint8_t* lidar_buf) const {
 if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
   // LEGACY profile has no packet_type - use 0 to code as 'legacy'
   return 0;
 }
 uint16_t res;
 std::memcpy(&res, lidar_buf + 0, sizeof(uint16_t));
 return res;
}

uint16_t packet_format::frame_id(const uint8_t* lidar_buf) const {
 if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
   return col_frame_id(nth_col(0, lidar_buf));
 }
 uint16_t res;
 std::memcpy(&res, lidar_buf + 2, sizeof(uint16_t));
 return res;
}

uint32_t packet_format::init_id(const uint8_t* lidar_buf) const {
 if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
   // LEGACY profile has no init_id - use 0 to code as 'legacy'
   return 0;
 }
 uint32_t res;
 std::memcpy(&res, lidar_buf + 4, sizeof(uint32_t));
 return res & 0x00ffffff;
}

uint64_t packet_format::prod_sn(const uint8_t* lidar_buf) const {
 if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
   // LEGACY profile has no prod_sn (serial number) - use 0 to code as
   // 'legacy'
   return 0;
 }
 uint64_t res;
 std::memcpy(&res, lidar_buf + 7, sizeof(uint64_t));
 return res & 0x000000ffffffffff;
}

uint16_t packet_format::countdown_thermal_shutdown(
       const uint8_t* lidar_buf) const {
 if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
   // LEGACY profile has no shutdown counter in packet header - use 0 for
   // 'normal operation'
   return 0;
 }
 uint16_t res;
 std::memcpy(&res, lidar_buf + 16, sizeof(uint8_t));
 return res;
}

uint16_t packet_format::countdown_shot_limiting(
       const uint8_t* lidar_buf) const {
 if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
   // LEGACY profile has no shot limiting countdown in packet header - use
   // 0 for 'normal operation'
   return 0;
 }
 uint16_t res;
 std::memcpy(&res, lidar_buf + 17, sizeof(uint8_t));
 return res;
}

uint8_t packet_format::thermal_shutdown(const uint8_t* lidar_buf) const {
 if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
   // LEGACY profile has no shutdown status in packet header - use 0 for
   // 'normal operation'
   return 0;
 }
 uint8_t res;
 std::memcpy(&res, lidar_buf + 18, sizeof(uint8_t));
 return res & 0x0f;
}

uint8_t packet_format::shot_limiting(const uint8_t* lidar_buf) const {
 if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
   // LEGACY profile has no shot limiting in packet header - use 0 for
   // 'normal operation'
   return 0;
 }
 uint8_t res;
 std::memcpy(&res, lidar_buf + 19, sizeof(uint8_t));
 return res & 0x0f;
}

const uint8_t* packet_format::footer(const uint8_t* lidar_buf) const {
 if (impl_->packet_footer_size == 0) return nullptr;
 return lidar_buf + impl_->packet_header_size +
        (columns_per_packet * impl_->col_size);
}

/* Measurement block access */

const uint8_t* packet_format::nth_col(int n, const uint8_t* lidar_buf) const {
 return lidar_buf + impl_->packet_header_size + (n * impl_->col_size);
}

uint32_t packet_format::col_status(const uint8_t* col_buf) const {
 uint32_t res;
 std::memcpy(&res, col_buf + impl_->status_offset, sizeof(uint32_t));
 if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
   return res;  // LEGACY was 32 bits of all 1s
 } else {
   return res & 0xffff;  // For eUDP packets, we want the last 16 bits
 }
}

uint64_t packet_format::col_timestamp(const uint8_t* col_buf) const {
 uint64_t res;
 std::memcpy(&res, col_buf + impl_->timestamp_offset, sizeof(uint64_t));
 return res;
}

uint16_t packet_format::col_measurement_id(const uint8_t* col_buf) const {
 uint16_t res;
 std::memcpy(&res, col_buf + impl_->measurement_id_offset, sizeof(uint16_t));
 return res;
}

uint16_t packet_format::col_frame_id(const uint8_t* col_buf) const {
 if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
   uint16_t res;
   std::memcpy(&res, col_buf + 10, sizeof(uint16_t));
   return res;
 } else {
   return 0;
 }
}

/* Channel data fields */

const uint8_t* packet_format::nth_px(int n, const uint8_t* col_buf) const {
 return col_buf + impl_->col_header_size + (n * impl_->channel_data_size);
}

template <typename T>
T packet_format::px_field(const uint8_t* px_buf, ChanField i) const {
 //FIXME(debug): remove this
// std::cout << "in px_field" << std::endl;
 const auto& f = impl_->fields.at(i);

 if (sizeof(T) < field_type_size(f.ty_tag))
   throw std::invalid_argument("Dest type too small for specified field");

 T res = 0;
 std::memcpy(&res, px_buf + f.offset, field_type_size(f.ty_tag));
 if (f.mask) res &= f.mask;
 if (f.shift > 0) res >>= f.shift;
 if (f.shift < 0) res <<= std::abs(f.shift);
 return res;
}

uint32_t packet_format::px_range(const uint8_t* px_buf) const {
 return px_field<uint32_t>(px_buf, ChanField::RANGE);
}

uint16_t packet_format::px_reflectivity(const uint8_t* px_buf) const {
 return px_field<uint16_t>(px_buf, ChanField::REFLECTIVITY);
}

uint16_t packet_format::px_signal(const uint8_t* px_buf) const {
 return px_field<uint16_t>(px_buf, ChanField::SIGNAL);
}

uint16_t packet_format::px_ambient(const uint8_t* px_buf) const {
 return px_field<uint16_t>(px_buf, ChanField::NEAR_IR);
}

/* IMU packet parsing */

uint64_t packet_format::imu_sys_ts(const uint8_t* imu_buf) {
 uint64_t res;
 std::memcpy(&res, imu_buf, sizeof(uint64_t));
 return res;
}

uint64_t packet_format::imu_accel_ts(const uint8_t* imu_buf) {
 uint64_t res;
 std::memcpy(&res, imu_buf + 8, sizeof(uint64_t));
 return res;
}

uint64_t packet_format::imu_gyro_ts(const uint8_t* imu_buf) {
 uint64_t res;
 std::memcpy(&res, imu_buf + 16, sizeof(uint64_t));
 return res;
}

float packet_format::imu_la_x(const uint8_t* imu_buf) {
 float res;
 std::memcpy(&res, imu_buf + 24, sizeof(float));
 return res;
}

float packet_format::imu_la_y(const uint8_t* imu_buf) {
 float res;
 std::memcpy(&res, imu_buf + 28, sizeof(float));
 return res;
}

float packet_format::imu_la_z(const uint8_t* imu_buf) {
 float res;
 std::memcpy(&res, imu_buf + 32, sizeof(float));
 return res;
}

float packet_format::imu_av_x(const uint8_t* imu_buf) {
 float res;
 std::memcpy(&res, imu_buf + 36, sizeof(float));
 return res;
}

float packet_format::imu_av_y(const uint8_t* imu_buf) {
 float res;
 std::memcpy(&res, imu_buf + 40, sizeof(float));
 return res;
}

float packet_format::imu_av_z(const uint8_t* imu_buf) {
 float res;
 std::memcpy(&res, imu_buf + 44, sizeof(float));
 return res;
}

const packet_format& get_format(const sensor_info& info) {
 using key = std::tuple<int, int, UDPProfileLidar>;
 static std::map<key, std::unique_ptr<packet_format>> cache{};
 static std::mutex cache_mx{};

 key k{info.format.pixels_per_column, info.format.columns_per_packet,
       info.format.udp_profile_lidar};

 std::lock_guard<std::mutex> lk{cache_mx};
 if (!cache.count(k)) {
   cache[k] = std::make_unique<packet_format>(info);
 }

 //FIXME(debug): remove this
// std::cout << "in get_format" << std::endl;
 return *cache.at(k);
}

}  // namespace sensor
}  // namespace ouster
