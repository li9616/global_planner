#ifndef _SINGLETON_
#define _SINGLETON_

#include <mutex>
#include <memory>

template <typename T>
class Singleton{
  public:
    template<typename ...Args>
    static std::shared_ptr<T> GetInstance(Args&&... args) {
        if (!m_pSington) {
            std::lock_guard<std::mutex> gLock(m_Mutex);
            if (m_pSington == nullptr) {
                m_pSington = std::make_shared<T>(std::forward<Args>(args)...);
            }
        }
        return m_pSington;
    }

    static void DesInstance() {
        if (m_pSington) {
            m_pSington.reset();
            m_pSington = nullptr;
        }
    }
  private:
    explicit Singleton();
    Singleton<T>(const Singleton<T>&) = delete;
    Singleton<T>& operator=(const Singleton<T>&) = delete;
    ~Singleton();

    static std::shared_ptr<T> m_pSington;
    static std::mutex m_Mutex;
};

template <typename T>
std::shared_ptr<T> Singleton<T>::m_pSington = nullptr;

template <typename T>
std::mutex Singleton<T>::m_Mutex;


#endif