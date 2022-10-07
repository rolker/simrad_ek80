#ifndef SIMRAD_EK80_FILE_FORMAT_H
#define SIMRAD_EK80_FILE_FORMAT_H

#include <cstdint>

/// \file
/// 
///         Vessel coordinate system (from EK80 Reference Manual)
///
///         The X-axis is the longitudinal direction of the vessel, and in
///         parallel with the deck. A positive value for X means that a sensor
///         or a reference point is located ahead of the reference point (origin).
///
///         The Y-axis is the transverse direction of the vessel, and in parallel
///         with the deck. A positive value for Y means that a sensor or a reference
///         point is located on the starboard side of the reference point (origin).
///
///         The Z-axis is vertical, and in parallel with the mast. A positive value
///         for Z means that a sensor or a new reference point is located under the
///         reference point (origin).
///
///         The vessel coordinate system follows the right-hand rule.
///
///         The rotation angle is defined to be positive for a rotation that is
///         clockwise when viewed by an observer looking along the rotation axis
///         from the origin of the coordinate system
///
///         X-axis
///         This is the main axis in the vessel’s forward (alongship) direction. Positive
///         values are forward relative to the origin. Positive rotation angle is starboard
///         side down.
///
///         Y-axis
///         This is the transverse (athwartships) direction. Positive values are toward
///         starboard relative to the origin. Positive rotation angle is bow up.
///
///         Z-axis
///         This is the vertical direction in parallel with the mast. Positive values are down
///         relative to the origin. Positive rotation angle is bow towards starboard.



namespace simrad
{

namespace file
{

enum TypeID: uint32_t
{
  XMLType = 0x304c4d58,
  NMEAType = 0x30454d4e,
  SampleType3 = 0x33574152,
  MRUType = 0x3055524d,
  FilterType = 0x314c4946,
  AnotationType = 0x30474154,
};

#pragma pack(1)

struct DatagramHeader
{
  int32_t DatagramType;
  ///  contains a 64-bit integer value stating the number of 100 nanosecond intervals since January 1, 1601. This is the internal ?filetime? used by the Windows NT operating system.
  uint64_t DateTime;
};

struct XMLDatagram
{
  constexpr static TypeID type = TypeID::XMLType;
  DatagramHeader header;
  char xml;
};

/// The NMEA datagrams contain original NMEA 0183 input
///                     sentences.
struct NMEADatagram
{
  constexpr static TypeID type = TypeID::NMEAType;
  DatagramHeader header;
  char nmea;
};

struct Datagram
{
  DatagramHeader DgHeader;
};

/// The annotation datagram contains comment text.
struct AnotationDatagram :Datagram
{
  constexpr static TypeID type = TypeID::AnotationType;
  char Text;
};

struct SampleDatagram3 :Datagram
{
  constexpr static TypeID type = TypeID::SampleType3;
  uint8_t ChannelID[128];
  uint16_t Datatype;
  int8_t Spare[2];
  int32_t Offset;
  int32_t Count;
  uint8_t Samples;
};

struct MRUDatagram: Datagram
{
  constexpr static TypeID type = TypeID::MRUType;
  float Heave;
  float Roll;
  float Pitch;
  float Heading;
};

struct FilterDatagram: Datagram
{
  constexpr static TypeID type = TypeID::FilterType;
  int16_t Stage;
  uint8_t Spare[2];
  int8_t FilterType;
  uint8_t ChannelID[128];
  int16_t NoOfCoefficients;
  int16_t DecimationFactor;
  float Coefficients;
};

#pragma pack()

}
}

#endif
