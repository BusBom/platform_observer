#include "json.hpp"
#include <fcgi_stdio.h>
#include <iostream>
#include <string>
#include <sys/mman.h>                 // POSIX shared memory
#include <sys/stat.h>                 // For mode constants
#include <fcntl.h>                    // For O_RDONLY
#include <unistd.h>                   // For close(), munmap
#include <string.h>                   // For strerror

using ordered_json = nlohmann::ordered_json;
using json = nlohmann::json;

// Shared memory constants (must match writer)
const char* SHM_SEQUENCE_NAME = "/busbom_sequence";
const size_t SHM_SEQUENCE_SIZE = 4096; // 4KB

int main() {
    while (FCGI_Accept() >= 0) {
        printf("Content-Type: application/json\r\n\r\n");
        
        ordered_json final_response;
        ordered_json sequence_data;
        int shm_fd = -1;
        char* shm_ptr = nullptr;

        try {
            shm_fd = shm_open(SHM_SEQUENCE_NAME, O_RDONLY, 0666);
            if (shm_fd == -1) {
                throw std::runtime_error(std::string("Failed to open shared memory: ") + strerror(errno));
            }

            shm_ptr = (char*)mmap(0, SHM_SEQUENCE_SIZE, PROT_READ, MAP_SHARED, shm_fd, 0);
            if (shm_ptr == MAP_FAILED) {
                throw std::runtime_error(std::string("Failed to map shared memory: ") + strerror(errno));
            }

            // Find the null terminator to get the actual string length
            size_t actual_len = strlen(shm_ptr);
            std::string shm_content = std::string(shm_ptr, actual_len);

            if (shm_content.empty()) {
                sequence_data = json::array(); // Empty array if shared memory is empty
            } else {
                sequence_data = json::parse(shm_content);
            }

        } catch (const std::exception& e) {
            // If shared memory read or parse fails, return an empty array for sequence
            sequence_data = json::array(); 
            // Optionally, log the error: std::cerr << "Error reading/parsing shared memory: " << e.what() << std::endl;
        }

        // Clean up shared memory resources
        if (shm_ptr != nullptr) {
            munmap(shm_ptr, SHM_SEQUENCE_SIZE);
        }
        if (shm_fd != -1) {
            close(shm_fd);
        }

        // Construct the final JSON response
        final_response["parkingPoint"] = 2;
        final_response["online"] = {0, 0, 0};
        final_response["sequence"] = sequence_data;

        std::cout << final_response.dump(4) << std::endl;
    };

    return 0;
};