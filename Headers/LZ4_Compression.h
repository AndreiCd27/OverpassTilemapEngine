#include <iostream>
#include <vector>
#include <fstream>
#include <cstring>
#include <lz4.h>

// Compresses data from a file and returns the compressed bytes
std::vector<char> CompressFileLZ4(const std::string& inputPath) {
    // Read the original binary file into memory
    std::ifstream file(inputPath, std::ios::binary | std::ios::ate);
    if (!file.is_open()) return {};

    std::streamsize uncompressedSize = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> srcBuffer(uncompressedSize);
    file.read(srcBuffer.data(), uncompressedSize);
    file.close();

    // Allocate buffer based on compression size
    // +4 bytes to store the original uncompressed size at the start of the file
    int CompressedSize = LZ4_compressBound(static_cast<int>(uncompressedSize));
    std::vector<char> dstBuffer(sizeof(uint32_t) + CompressedSize);

    // Store uncompressed size as metadata header
    uint32_t sizeHeader = static_cast<uint32_t>(uncompressedSize);
    std::memcpy(dstBuffer.data(), &sizeHeader, sizeof(uint32_t));

    // Perform LZ4 Compression
    int actualCompressedSize = LZ4_compress_default(
        srcBuffer.data(), 
        dstBuffer.data() + sizeof(uint32_t), 
        static_cast<int>(uncompressedSize), 
        CompressedSize
    );

    if (actualCompressedSize <= 0) {
        throw std::runtime_error("LZ4 compression failed!");
    }

    // Shrink the vector to match the exact compressed file size
    dstBuffer.resize(sizeof(uint32_t) + actualCompressedSize);
    return dstBuffer;
}

// Decompresses an LZ4-compressed file
std::vector<char> DecompressFileLZ4(const std::string& compressedPath) {
    // Read compressed file
    std::ifstream file(compressedPath, std::ios::binary | std::ios::ate);
    if (!file.is_open()) return {};

    std::streamsize fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> srcBuffer(fileSize);
    file.read(srcBuffer.data(), fileSize);
    file.close();

    // Extract original uncompressed size from the 4-byte header
    uint32_t uncompressedSize = 0;
    std::memcpy(&uncompressedSize, srcBuffer.data(), sizeof(uint32_t));

    // Allocate memory for te uncompressed data
    std::vector<char> dstBuffer(uncompressedSize);

    // Perform LZ4 Decompression
    int result = LZ4_decompress_safe(
        srcBuffer.data() + sizeof(uint32_t), // Start past the header
        dstBuffer.data(), 
        static_cast<int>(fileSize) - sizeof(uint32_t), 
        static_cast<int>(uncompressedSize)
    );

    if (result < 0) {
        throw std::runtime_error("LZ4 decompression failed! Corrupted data.");
    }

    return dstBuffer;
}
