#pragma once

#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>
#include <boost/crc.hpp>

using boost::asio::ip::tcp;

void calculate_crc32(const char* data, int length = -1);

/* ============================================================================================================ */

void lz4_concat_noprime(tcp::socket &socket);
void lz4_concat_prime(tcp::socket &socket);
void lz4_noconcat_noprime(tcp::socket &socket);
void lz4_noconcat_prime(tcp::socket &socket);

/* ============================================================================================================ */

void zlib_concat_noprime(tcp::socket &socket);
void zlib_concat_prime(tcp::socket &socket);
void zlib_noconcat_noprime(tcp::socket &socket);
void zlib_noconcat_prime(tcp::socket &socket);

/* ============================================================================================================ */

void zstd_concat_noprime(tcp::socket &socket);
void zstd_concat_prime(tcp::socket &socket);
void zstd_noconcat_noprime(tcp::socket &socket);
void zstd_noconcat_prime(tcp::socket &socket);

/* ============================================================================================================ */

void zstd_gray_concat_noprime(tcp::socket &socket);
void zstd_gray_concat_prime(tcp::socket &socket);
void zstd_gray_noconcat_noprime(tcp::socket &socket);
void zstd_gray_noconcat_prime(tcp::socket &socket);

/* ============================================================================================================ */