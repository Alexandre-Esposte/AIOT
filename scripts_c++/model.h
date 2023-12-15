#ifndef UUID140319963469072
#define UUID140319963469072

#include <EloquentTinyML.h>
#include <eloquent_tinyml/tensorflow.h>

#ifdef __has_attribute
#define HAVE_ATTRIBUTE(x) __has_attribute(x)
#else
#define HAVE_ATTRIBUTE(x) 0
#endif
#if HAVE_ATTRIBUTE(aligned) || (defined(__GNUC__) && !defined(__clang__))
#define DATA_ALIGN_ATTRIBUTE __attribute__((aligned(4)))
#else
#define DATA_ALIGN_ATTRIBUTE
#endif

#ifndef ARENA_SIZE
#define ARENA_SIZE 4096
#endif

/** model size = 3612 bytes **/
const unsigned char modelData[] DATA_ALIGN_ATTRIBUTE = { 0x1c, 0x00, 0x00, 0x00, 0x54, 0x46, 0x4c, 0x33, 0x14, 0x00, 0x20, 0x00, 0x1c, 0x00, 0x18, 0x00, 0x14, 0x00, 0x10, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x08, 0x00, 0x04, 0x00, 0x14, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x8c, 0x00, 0x00, 0x00, 0xe4, 0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x70, 0x06, 0x00, 0x00, 0xb8, 0x0d, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x10, 0x00, 0x0c, 0x00, 0x08, 0x00, 0x04, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x73, 0x65, 0x72, 0x76, 0x69, 0x6e, 0x67, 0x5f, 0x64, 0x65, 0x66, 0x61, 0x75, 0x6c, 0x74, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x9c, 0xff, 0xff, 0xff, 0x0a, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x5f, 0x32, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x82, 0xf9, 0xff, 0xff, 0x04, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x69, 0x6e, 0x70, 0x75, 0x74, 0x5f, 0x31, 0x00, 0x02, 0x00, 0x00, 0x00, 0x34, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0xdc, 0xff, 0xff, 0xff, 0x0d, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x43, 0x4f, 0x4e, 0x56, 0x45, 0x52, 0x53, 0x49, 0x4f, 0x4e, 0x5f, 0x4d, 0x45, 0x54, 0x41, 0x44, 0x41, 0x54, 0x41, 0x00, 0x08, 0x00, 0x0c, 0x00, 0x08, 0x00, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x6d, 0x69, 0x6e, 0x5f, 0x72, 0x75, 0x6e, 0x74, 0x69, 0x6d, 0x65, 0x5f, 0x76, 0x65, 0x72, 0x73, 0x69, 0x6f, 0x6e, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x78, 0x05, 0x00, 0x00, 0x70, 0x05, 0x00, 0x00, 0x40, 0x05, 0x00, 0x00, 0x30, 0x02, 0x00, 0x00, 0x08, 0x02, 0x00, 0x00, 0x38, 0x01, 0x00, 0x00, 0x18, 0x01, 0x00, 0x00, 0xa8, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x98, 0x00, 0x00, 0x00, 0x90, 0x00, 0x00, 0x00, 0x88, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x2e, 0xfa, 0xff, 0xff, 0x04, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x08, 0x00, 0x0e, 0x00, 0x08, 0x00, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x08, 0x00, 0x04, 0x00, 0x06, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x10, 0x00, 0x0c, 0x00, 0x08, 0x00, 0x04, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x32, 0x2e, 0x31, 0x35, 0x2e, 0x30, 0x00, 0x00, 0x8e, 0xfa, 0xff, 0xff, 0x04, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x31, 0x2e, 0x35, 0x2e, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0xf4, 0xff, 0xff, 0x10, 0xf4, 0xff, 0xff, 0x14, 0xf4, 0xff, 0xff, 0x18, 0xf4, 0xff, 0xff, 0xba, 0xfa, 0xff, 0xff, 0x04, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x70, 0x9a, 0xc8, 0x3e, 0x6b, 0x5c, 0x99, 0x3e, 0x8e, 0x07, 0xf7, 0xbd, 0x76, 0xf6, 0x28, 0xbf, 0xc2, 0x94, 0x6e, 0xbc, 0x49, 0x79, 0x6e, 0x3f, 0x57, 0xd6, 0x8b, 0x3e, 0x74, 0x2a, 0x78, 0x3e, 0x26, 0x56, 0x2c, 0xbf, 0x2e, 0x4d, 0x28, 0x3f, 0xf2, 0x07, 0x5e, 0x3e, 0xdc, 0x3f, 0x0a, 0x3f, 0x3a, 0x90, 0x0b, 0x3f, 0x5d, 0xcf, 0x83, 0xbe, 0xc7, 0x4e, 0x5e, 0xbc, 0xa7, 0xe5, 0x8d, 0xbe, 0x46, 0xd1, 0x7d, 0x3c, 0x70, 0xcc, 0xf2, 0xbe, 0xfc, 0xc6, 0x6c, 0xbf, 0xe0, 0xf3, 0xf9, 0x3e, 0x2f, 0xac, 0x7d, 0x3e, 0x02, 0xe2, 0xfc, 0x3e, 0xad, 0x37, 0xfe, 0x3e, 0x57, 0xac, 0x37, 0xbe, 0x26, 0xfb, 0xff, 0xff, 0x04, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0xce, 0x72, 0xc6, 0xbe, 0xda, 0x4f, 0x98, 0xbe, 0x67, 0xfd, 0xda, 0x3e, 0x9b, 0x2e, 0x17, 0x3e, 0x42, 0xfb, 0xff, 0xff, 0x04, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x8b, 0x93, 0xa4, 0xbf, 0x7b, 0x25, 0x63, 0x3f, 0x43, 0xa7, 0x9b, 0xbf, 0xa2, 0xf1, 0xb1, 0xbe, 0xb5, 0x1f, 0x8e, 0xbf, 0x05, 0x7b, 0xe2, 0xbd, 0xa5, 0xbd, 0xf2, 0x3e, 0x27, 0xa8, 0x94, 0xbe, 0xd1, 0x33, 0x60, 0x3f, 0x80, 0x84, 0x01, 0xbf, 0x48, 0xa3, 0xf5, 0xbd, 0x6d, 0x11, 0x79, 0xbf, 0xdf, 0x62, 0x88, 0x3f, 0x70, 0x3c, 0x12, 0x3e, 0x0b, 0x4e, 0xf6, 0x3f, 0xdc, 0x83, 0xf7, 0x3e, 0x34, 0x04, 0xaa, 0x3d, 0x0b, 0x54, 0x57, 0xbf, 0x9a, 0xdb, 0x96, 0x3f, 0xb8, 0x52, 0xb5, 0xbf, 0x83, 0x3b, 0x8e, 0xbe, 0xa7, 0xaa, 0xc5, 0x3e, 0x0b, 0xaf, 0x60, 0xbf, 0xbb, 0x53, 0x5d, 0x3e, 0xe0, 0xd1, 0x3a, 0xbf, 0x00, 0x99, 0x23, 0x3e, 0x87, 0xb1, 0xa7, 0x3f, 0x1f, 0x05, 0xa3, 0xbf, 0xf5, 0x8d, 0xfe, 0xbe, 0xfd, 0x40, 0x6e, 0x3e, 0xa3, 0xb4, 0xfc, 0x3f, 0xd2, 0x3a, 0x1a, 0xbf, 0x62, 0xff, 0x85, 0x3f, 0x92, 0x9a, 0x0d, 0xbf, 0x5b, 0x58, 0x76, 0x3f, 0x52, 0xab, 0x8a, 0x3e, 0x7c, 0xd1, 0x97, 0x3f, 0xb2, 0x56, 0x2f, 0x3f, 0x65, 0xa2, 0xcb, 0xbf, 0xb3, 0x37, 0x0c, 0xbf, 0x8c, 0x4d, 0x80, 0xbf, 0x0a, 0xa5, 0xbd, 0x3f, 0x04, 0x5d, 0xa1, 0x3c, 0xd1, 0x76, 0xdd, 0x3f, 0xf0, 0xda, 0x91, 0xbe, 0xc7, 0xce, 0x21, 0x3e, 0x9c, 0xc7, 0xfc, 0x3f, 0xef, 0xa3, 0x04, 0xbf, 0x0e, 0xfc, 0xff, 0xff, 0x04, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x86, 0xfb, 0x12, 0x40, 0xeb, 0x99, 0xb5, 0xbf, 0xde, 0xde, 0xb1, 0x3e, 0x0b, 0x24, 0x55, 0x3f, 0x8a, 0x55, 0x7d, 0xbf, 0x32, 0xbf, 0x80, 0x3e, 0x32, 0xfc, 0xff, 0xff, 0x04, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x78, 0x4d, 0x22, 0xbf, 0x2c, 0xf7, 0xaf, 0x3f, 0xfe, 0xf8, 0x9f, 0x3f, 0xec, 0x61, 0xff, 0x3f, 0x97, 0x61, 0xed, 0xbe, 0xfe, 0x9b, 0x2c, 0x40, 0x8f, 0x8a, 0xf4, 0x3e, 0x3e, 0xd1, 0x5f, 0x40, 0x0c, 0x1e, 0xda, 0xbf, 0xeb, 0x80, 0x2c, 0x40, 0x56, 0x26, 0x63, 0x40, 0x2b, 0x6c, 0x1b, 0x40, 0x36, 0xa6, 0x41, 0x40, 0x52, 0x4f, 0x7d, 0x3f, 0xa9, 0xb5, 0xc5, 0x3f, 0xd8, 0x01, 0xb3, 0x40, 0x73, 0x64, 0x8e, 0xbf, 0xaf, 0x83, 0xff, 0xbe, 0x32, 0xa1, 0xbb, 0xbf, 0x01, 0xc2, 0x9e, 0xbf, 0x39, 0x0e, 0xd4, 0xbd, 0x34, 0xdf, 0x97, 0xbf, 0x06, 0x38, 0xbf, 0xbf, 0xb8, 0xd1, 0x00, 0x3f, 0xd1, 0x40, 0x53, 0x3f, 0xc4, 0xbb, 0x16, 0x40, 0xba, 0x3b, 0x00, 0x40, 0x7a, 0x65, 0xf2, 0x3f, 0xe1, 0x7b, 0x9a, 0xbf, 0xf8, 0x78, 0xa7, 0xbf, 0x96, 0x82, 0x54, 0xbf, 0x85, 0xfa, 0xcb, 0x3e, 0x5a, 0x85, 0x04, 0xbf, 0x96, 0xb4, 0xbb, 0x3f, 0x34, 0x6d, 0x3f, 0xbf, 0xcf, 0xba, 0xa8, 0x3e, 0x84, 0x28, 0x4e, 0xc0, 0x32, 0x7f, 0x6f, 0xbf, 0x53, 0xd9, 0x50, 0xbf, 0xdf, 0xe2, 0x32, 0x3f, 0xa1, 0xba, 0x66, 0x3f, 0x11, 0x53, 0x5f, 0x3f, 0xf5, 0x48, 0xa9, 0x3f, 0x09, 0x11, 0x65, 0x40, 0xed, 0x56, 0xa8, 0x3f, 0xc9, 0x7a, 0x5c, 0xc0, 0xc0, 0x4c, 0xd1, 0xbf, 0xe0, 0x22, 0x80, 0xc0, 0xba, 0x66, 0x2c, 0xc0, 0x30, 0xfc, 0x43, 0xbe, 0x42, 0x4a, 0xb6, 0x3f, 0xe3, 0x0b, 0x49, 0x40, 0x29, 0x38, 0x79, 0xbf, 0x10, 0x14, 0x09, 0x40, 0xfc, 0xb1, 0x5b, 0x3e, 0x15, 0x81, 0xf4, 0xbf, 0xdb, 0xc9, 0x88, 0x3f, 0x93, 0xd8, 0x06, 0xc0, 0x29, 0xb7, 0xdc, 0xbf, 0x5f, 0x68, 0x3f, 0xbf, 0x60, 0xc7, 0x4f, 0x40, 0xb6, 0xa8, 0x8b, 0xbf, 0xaf, 0xb9, 0xc8, 0x3f, 0xd7, 0x81, 0x90, 0x40, 0x40, 0x06, 0x01, 0xc1, 0xc8, 0x27, 0x04, 0x40, 0x83, 0x8a, 0x9e, 0x3d, 0x4b, 0xa6, 0x30, 0x40, 0xc6, 0x50, 0x94, 0xbf, 0xbe, 0x9c, 0x6a, 0x3f, 0xed, 0x3c, 0x5f, 0xbe, 0x09, 0x89, 0xbc, 0xbe, 0x46, 0xfd, 0x11, 0x3e, 0x17, 0xa3, 0xda, 0x3e, 0xf7, 0x0c, 0xb8, 0xbf, 0x83, 0x15, 0x41, 0x3f, 0x03, 0x8e, 0xfc, 0x3f, 0x70, 0xb8, 0xa5, 0x3d, 0xee, 0x73, 0x0b, 0x3e, 0x29, 0xdb, 0x3e, 0xc0, 0x4a, 0x26, 0x05, 0xbf, 0xaf, 0x35, 0xa9, 0xbf, 0x9e, 0xa0, 0xbe, 0xc0, 0x9e, 0xf0, 0x6a, 0x40, 0xcb, 0x34, 0xad, 0x3f, 0xca, 0xd8, 0x14, 0xbf, 0x95, 0x9d, 0x1f, 0x3f, 0xdd, 0x25, 0xdd, 0x3f, 0x82, 0x08, 0x71, 0xbf, 0x2f, 0x16, 0x3f, 0xc0, 0xec, 0x5b, 0x4e, 0x3f, 0x61, 0x63, 0x47, 0x3f, 0x24, 0x19, 0x2e, 0x40, 0xf3, 0x9f, 0x1d, 0x40, 0x7d, 0x95, 0x35, 0x40, 0xd3, 0x4e, 0xcc, 0xbf, 0xbf, 0x30, 0x80, 0x3f, 0xf3, 0x99, 0xe4, 0x3f, 0x30, 0x90, 0xf9, 0x3f, 0x65, 0xf7, 0xb0, 0xbe, 0x45, 0x79, 0xe4, 0x3f, 0x00, 0x9d, 0xf5, 0xbe, 0x3f, 0x9b, 0x04, 0x40, 0xeb, 0x5a, 0x31, 0xc0, 0x9a, 0xc0, 0xa0, 0x3f, 0x84, 0xa8, 0x4c, 0xc0, 0x4f, 0x43, 0x3c, 0xc0, 0xa0, 0x16, 0x46, 0xc0, 0x92, 0x1a, 0x40, 0xbf, 0x9e, 0x05, 0x38, 0xbe, 0x09, 0xf3, 0xa1, 0x3f, 0xa5, 0xcb, 0x05, 0x3f, 0xe7, 0xd6, 0x24, 0x40, 0x89, 0xef, 0xca, 0x3f, 0xc5, 0x0a, 0x6d, 0xc0, 0x16, 0xec, 0x3d, 0xc0, 0xf5, 0xdb, 0x87, 0x3e, 0xc4, 0x88, 0x91, 0xbf, 0x05, 0xcf, 0x18, 0x3f, 0xe4, 0x38, 0x91, 0xbf, 0xb2, 0x5d, 0xb9, 0xbf, 0xe8, 0xda, 0x09, 0x40, 0x2e, 0xa4, 0x2f, 0x40, 0xf2, 0x9a, 0x67, 0x40, 0x1d, 0xbc, 0xe1, 0xbe, 0x14, 0xcf, 0x5c, 0x3f, 0x93, 0x64, 0xf6, 0xbd, 0xd0, 0x5e, 0xb1, 0xbf, 0x16, 0x34, 0x1b, 0xbf, 0xe8, 0xd2, 0x1e, 0xc0, 0x9f, 0xab, 0x71, 0xc0, 0x2c, 0x86, 0x0b, 0xc0, 0x65, 0xd5, 0x09, 0x40, 0x29, 0xbe, 0x1d, 0xbf, 0x3b, 0xbf, 0xda, 0x3f, 0x63, 0xfd, 0x96, 0x3e, 0x1f, 0xf8, 0x89, 0xc0, 0xe6, 0x24, 0x3e, 0xbe, 0x49, 0x7f, 0x16, 0xc0, 0x32, 0xb0, 0x3c, 0xc0, 0x70, 0xc4, 0x00, 0xbf, 0x57, 0xcd, 0x4d, 0xbc, 0x96, 0x3e, 0x1b, 0xc0, 0x13, 0x00, 0x10, 0xc0, 0xde, 0x70, 0x99, 0xbe, 0xfb, 0xe4, 0x19, 0xbe, 0x86, 0x27, 0x23, 0xbc, 0xf4, 0x94, 0xed, 0x3f, 0x49, 0x96, 0xbd, 0x3f, 0x2c, 0xb5, 0x45, 0x3e, 0xcc, 0xdb, 0x3f, 0x3f, 0xe0, 0xbe, 0x79, 0xbf, 0x5e, 0xfd, 0x55, 0xbf, 0xbc, 0x90, 0xd0, 0x40, 0xdf, 0x52, 0x9a, 0xbf, 0x2b, 0x1c, 0xf2, 0xbf, 0xb8, 0x04, 0x24, 0xbf, 0xb4, 0x3e, 0x03, 0x3f, 0xdb, 0xec, 0x24, 0x3f, 0x22, 0xe1, 0xd9, 0x40, 0xa5, 0xb5, 0xeb, 0xc0, 0x05, 0xde, 0x68, 0x3f, 0xdc, 0x4d, 0x9f, 0x3f, 0x5f, 0xa6, 0x25, 0x40, 0xde, 0xb4, 0x12, 0xbf, 0x32, 0x19, 0x22, 0xbf, 0x4c, 0x89, 0xbd, 0xbe, 0x17, 0xba, 0xec, 0xbf, 0xbe, 0xfc, 0x15, 0x3f, 0x80, 0xb0, 0x08, 0xc0, 0xf7, 0x72, 0x1a, 0x3f, 0xcf, 0xfc, 0x97, 0xbf, 0x69, 0xab, 0x94, 0x3d, 0x6d, 0x22, 0xbb, 0xbf, 0x5c, 0x92, 0xb7, 0xbf, 0xb5, 0x94, 0x69, 0x40, 0xa1, 0x18, 0xc3, 0x3f, 0xeb, 0x7b, 0xa2, 0x40, 0x10, 0x7d, 0xf7, 0xbf, 0x43, 0x91, 0x31, 0x3f, 0x90, 0x51, 0xb2, 0x3f, 0x09, 0x5a, 0xdd, 0xbf, 0x04, 0xb4, 0x9e, 0xbf, 0x00, 0x3c, 0xbe, 0x40, 0x01, 0x1c, 0x9e, 0x40, 0x01, 0x9b, 0x7d, 0x3f, 0x8c, 0x67, 0x74, 0xbf, 0x06, 0xc4, 0x83, 0xc0, 0x19, 0x23, 0x8c, 0xbf, 0xde, 0x12, 0xa9, 0x3f, 0xb8, 0xe5, 0x0b, 0xbf, 0x4d, 0x18, 0x00, 0x40, 0x3e, 0xff, 0xff, 0xff, 0x04, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x48, 0x76, 0xe0, 0xc1, 0x82, 0x6c, 0xa3, 0xc1, 0x50, 0x30, 0x93, 0xc1, 0x2e, 0x3d, 0xad, 0xc1, 0x0c, 0x15, 0xe2, 0xc1, 0x29, 0x87, 0x0d, 0xc2, 0x8d, 0xc5, 0x3d, 0xbf, 0x8e, 0x84, 0x57, 0x41, 0xcc, 0xf8, 0xff, 0xff, 0xd0, 0xf8, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x4d, 0x4c, 0x49, 0x52, 0x20, 0x43, 0x6f, 0x6e, 0x76, 0x65, 0x72, 0x74, 0x65, 0x64, 0x2e, 0x00, 0x01, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x18, 0x00, 0x14, 0x00, 0x10, 0x00, 0x0c, 0x00, 0x08, 0x00, 0x04, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x24, 0x01, 0x00, 0x00, 0x28, 0x01, 0x00, 0x00, 0x2c, 0x01, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x6d, 0x61, 0x69, 0x6e, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0xcc, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x1a, 0x00, 0x14, 0x00, 0x10, 0x00, 0x0c, 0x00, 0x0b, 0x00, 0x04, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x1c, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x08, 0x00, 0x04, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x9a, 0xff, 0xff, 0xff, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x0c, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x94, 0xf9, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0xca, 0xff, 0xff, 0xff, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x10, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0xba, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x16, 0x00, 0x00, 0x00, 0x10, 0x00, 0x0c, 0x00, 0x0b, 0x00, 0x04, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x18, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x08, 0x00, 0x07, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x94, 0x05, 0x00, 0x00, 0x18, 0x05, 0x00, 0x00, 0x44, 0x04, 0x00, 0x00, 0xd8, 0x03, 0x00, 0x00, 0x10, 0x03, 0x00, 0x00, 0xbc, 0x02, 0x00, 0x00, 0x74, 0x02, 0x00, 0x00, 0xa4, 0x01, 0x00, 0x00, 0xcc, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0xae, 0xfa, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0x14, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x34, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x04, 0x00, 0x00, 0x00, 0x98, 0xfa, 0xff, 0xff, 0x19, 0x00, 0x00, 0x00, 0x53, 0x74, 0x61, 0x74, 0x65, 0x66, 0x75, 0x6c, 0x50, 0x61, 0x72, 0x74, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x65, 0x64, 0x43, 0x61, 0x6c, 0x6c, 0x3a, 0x30, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x06, 0xfb, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0x14, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x04, 0x00, 0x00, 0x00, 0xf0, 0xfa, 0xff, 0xff, 0x2a, 0x00, 0x00, 0x00, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x5f, 0x32, 0x2f, 0x4d, 0x61, 0x74, 0x4d, 0x75, 0x6c, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x5f, 0x32, 0x2f, 0x42, 0x69, 0x61, 0x73, 0x41, 0x64, 0x64, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x6e, 0xfb, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0x14, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0xb0, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x06, 0x00, 0x00, 0x00, 0x58, 0xfb, 0xff, 0xff, 0x96, 0x00, 0x00, 0x00, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x62, 0x61, 0x74, 0x63, 0x68, 0x5f, 0x6e, 0x6f, 0x72, 0x6d, 0x61, 0x6c, 0x69, 0x7a, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x5f, 0x31, 0x2f, 0x62, 0x61, 0x74, 0x63, 0x68, 0x6e, 0x6f, 0x72, 0x6d, 0x2f, 0x6d, 0x75, 0x6c, 0x5f, 0x31, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x62, 0x61, 0x74, 0x63, 0x68, 0x5f, 0x6e, 0x6f, 0x72, 0x6d, 0x61, 0x6c, 0x69, 0x7a, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x5f, 0x31, 0x2f, 0x62, 0x61, 0x74, 0x63, 0x68, 0x6e, 0x6f, 0x72, 0x6d, 0x2f, 0x61, 0x64, 0x64, 0x5f, 0x31, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x5f, 0x31, 0x2f, 0x4d, 0x61, 0x74, 0x4d, 0x75, 0x6c, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x5f, 0x31, 0x2f, 0x52, 0x65, 0x6c, 0x75, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x5f, 0x31, 0x2f, 0x42, 0x69, 0x61, 0x73, 0x41, 0x64, 0x64, 0x31, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x42, 0xfc, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0x14, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0xa8, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x08, 0x00, 0x00, 0x00, 0x2c, 0xfc, 0xff, 0xff, 0x8c, 0x00, 0x00, 0x00, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x62, 0x61, 0x74, 0x63, 0x68, 0x5f, 0x6e, 0x6f, 0x72, 0x6d, 0x61, 0x6c, 0x69, 0x7a, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x2f, 0x62, 0x61, 0x74, 0x63, 0x68, 0x6e, 0x6f, 0x72, 0x6d, 0x2f, 0x6d, 0x75, 0x6c, 0x5f, 0x31, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x62, 0x61, 0x74, 0x63, 0x68, 0x5f, 0x6e, 0x6f, 0x72, 0x6d, 0x61, 0x6c, 0x69, 0x7a, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x2f, 0x62, 0x61, 0x74, 0x63, 0x68, 0x6e, 0x6f, 0x72, 0x6d, 0x2f, 0x61, 0x64, 0x64, 0x5f, 0x31, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x2f, 0x4d, 0x61, 0x74, 0x4d, 0x75, 0x6c, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x2f, 0x52, 0x65, 0x6c, 0x75, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x2f, 0x42, 0x69, 0x61, 0x73, 0x41, 0x64, 0x64, 0x31, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x86, 0xfd, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0xe8, 0xfc, 0xff, 0xff, 0x14, 0x00, 0x00, 0x00, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x5f, 0x32, 0x2f, 0x4d, 0x61, 0x74, 0x4d, 0x75, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0xca, 0xfd, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x34, 0x00, 0x00, 0x00, 0x2c, 0xfd, 0xff, 0xff, 0x24, 0x00, 0x00, 0x00, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x5f, 0x32, 0x2f, 0x42, 0x69, 0x61, 0x73, 0x41, 0x64, 0x64, 0x2f, 0x52, 0x65, 0x61, 0x64, 0x56, 0x61, 0x72, 0x69, 0x61, 0x62, 0x6c, 0x65, 0x4f, 0x70, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x1a, 0xfe, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0xa4, 0x00, 0x00, 0x00, 0x7c, 0xfd, 0xff, 0xff, 0x95, 0x00, 0x00, 0x00, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x62, 0x61, 0x74, 0x63, 0x68, 0x5f, 0x6e, 0x6f, 0x72, 0x6d, 0x61, 0x6c, 0x69, 0x7a, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x5f, 0x31, 0x2f, 0x62, 0x61, 0x74, 0x63, 0x68, 0x6e, 0x6f, 0x72, 0x6d, 0x2f, 0x6d, 0x75, 0x6c, 0x5f, 0x31, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x62, 0x61, 0x74, 0x63, 0x68, 0x5f, 0x6e, 0x6f, 0x72, 0x6d, 0x61, 0x6c, 0x69, 0x7a, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x5f, 0x31, 0x2f, 0x62, 0x61, 0x74, 0x63, 0x68, 0x6e, 0x6f, 0x72, 0x6d, 0x2f, 0x61, 0x64, 0x64, 0x5f, 0x31, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x5f, 0x31, 0x2f, 0x4d, 0x61, 0x74, 0x4d, 0x75, 0x6c, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x5f, 0x31, 0x2f, 0x52, 0x65, 0x6c, 0x75, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x5f, 0x31, 0x2f, 0x42, 0x69, 0x61, 0x73, 0x41, 0x64, 0x64, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0xde, 0xfe, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x4c, 0x00, 0x00, 0x00, 0x40, 0xfe, 0xff, 0xff, 0x3d, 0x00, 0x00, 0x00, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x5f, 0x31, 0x2f, 0x4d, 0x61, 0x74, 0x4d, 0x75, 0x6c, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x5f, 0x31, 0x2f, 0x52, 0x65, 0x6c, 0x75, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x5f, 0x31, 0x2f, 0x42, 0x69, 0x61, 0x73, 0x41, 0x64, 0x64, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x46, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x98, 0x00, 0x00, 0x00, 0xa8, 0xfe, 0xff, 0xff, 0x8b, 0x00, 0x00, 0x00, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x62, 0x61, 0x74, 0x63, 0x68, 0x5f, 0x6e, 0x6f, 0x72, 0x6d, 0x61, 0x6c, 0x69, 0x7a, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x2f, 0x62, 0x61, 0x74, 0x63, 0x68, 0x6e, 0x6f, 0x72, 0x6d, 0x2f, 0x6d, 0x75, 0x6c, 0x5f, 0x31, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x62, 0x61, 0x74, 0x63, 0x68, 0x5f, 0x6e, 0x6f, 0x72, 0x6d, 0x61, 0x6c, 0x69, 0x7a, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x2f, 0x62, 0x61, 0x74, 0x63, 0x68, 0x6e, 0x6f, 0x72, 0x6d, 0x2f, 0x61, 0x64, 0x64, 0x5f, 0x31, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x2f, 0x4d, 0x61, 0x74, 0x4d, 0x75, 0x6c, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x2f, 0x52, 0x65, 0x6c, 0x75, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x2f, 0x42, 0x69, 0x61, 0x73, 0x41, 0x64, 0x64, 0x00, 0x02, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x00, 0x18, 0x00, 0x14, 0x00, 0x00, 0x00, 0x10, 0x00, 0x0c, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x78, 0xff, 0xff, 0xff, 0x37, 0x00, 0x00, 0x00, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x2f, 0x4d, 0x61, 0x74, 0x4d, 0x75, 0x6c, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x2f, 0x52, 0x65, 0x6c, 0x75, 0x3b, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x2f, 0x42, 0x69, 0x61, 0x73, 0x41, 0x64, 0x64, 0x00, 0x01, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x00, 0x1c, 0x00, 0x18, 0x00, 0x00, 0x00, 0x14, 0x00, 0x10, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x07, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x14, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x18, 0x00, 0x00, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x19, 0x00, 0x00, 0x00, 0x73, 0x65, 0x72, 0x76, 0x69, 0x6e, 0x67, 0x5f, 0x64, 0x65, 0x66, 0x61, 0x75, 0x6c, 0x74, 0x5f, 0x69, 0x6e, 0x70, 0x75, 0x74, 0x5f, 0x31, 0x3a, 0x30, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0xf4, 0xff, 0xff, 0xff, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x0c, 0x00, 0x0c, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09 };

/**
 * Wrapper around the EloquentTinyML library
 */
template<uint32_t arenaSize>
class TensorFlowPorter {
    public:
        Eloquent::TinyML::TensorFlow::AllOpsTensorFlow<24, 4, arenaSize> tf;

        /**
         * Init model
         */
        bool begin() {
            return tf.begin(modelData);
        }

        /**
         * Proxy
         */
        uint8_t predict(uint8_t *input, uint8_t *output = NULL) {
            return tf.predict(input, output);
        }

        /**
         * Proxy
         */
        int8_t predict(int8_t *input, int8_t *output = NULL) {
            return tf.predict(input, output);
        }

        /**
         * Proxy
         */
        float predict(float *input, float *output = NULL) {
            return tf.predict(input, output);
        }

        /**
         * Proxy
         */
        template<typename T>
        uint8_t predictClass(T *input) {
            return tf.predictClass(input);
        }

        /**
         * Proxy
         */
        float getScoreAt(uint8_t index) {
            return tf.getScoreAt(index);
        }

        /**
         * Proxy
         */
        String getErrorMessage() {
            return tf.getErrorMessage();
        }
};



TensorFlowPorter<ARENA_SIZE> model;


#endif

