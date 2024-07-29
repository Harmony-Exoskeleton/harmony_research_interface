#ifndef HARMONY_SHARED_MEMORY_MANAGER_H
#define HARMONY_SHARED_MEMORY_MANAGER_H

#include <sys/ipc.h>
#include <sys/shm.h>

namespace harmony {

template<typename T>
class SharedMemoryManager {
public:
    // Setting isMemoryOwner to true results in the memory being marked for destruction when
    // the manager detaches from the memory (typically in the destructor). Clients that do not
    // own the memory should set isMemoryOwner to false.
    explicit SharedMemoryManager(int id, bool isMemoryOwner)
        : key(ftok("/bin", id))
        , isMemoryOwner(isMemoryOwner) {}

    ~SharedMemoryManager() { detachSharedMemory(); }

    SharedMemoryManager(const SharedMemoryManager&) = delete;
    SharedMemoryManager& operator=(const SharedMemoryManager&) = delete;

    bool init() {
        if (isInitialized) { return true; }

        // shmget returns an identifier in shmid
        T temp;
        shmid = shmget(key, sizeof(temp), 0666 | IPC_CREAT);

        // shmat to attach to shared memory
        data = static_cast<T*>(shmat(shmid, nullptr, 0));

        bool success = shmid != -1 && key != -1;

        if (success) { isInitialized = true; }
        return success;
    }

    void detachSharedMemory() {
        if (isInitialized) {
            isInitialized = false;
            shmdt(data);
            if (isMemoryOwner) { shmctl(shmid, IPC_RMID, nullptr); }
        }
    }

    T* data;

private:
    key_t key;
    int shmid = 0;
    bool isInitialized = false;
    bool isMemoryOwner;
};

} // namespace harmony

#endif
