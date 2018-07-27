#ifndef _SINGLETON_
#define _SINGLETON_

#include <mutex>
#include <memory>

template <typename T>
class Singleton{
  public:
    template<typename ...Args>
    static boost::shared_ptr<T> GetInstance(Args&&... args) {
        if (!m_pSington) {
            boost::lock_guard<boost::mutex> gLock(m_Mutex);
            if (m_pSington == nullptr) {
                m_pSington = boost::make_shared<T>(boost::forward<Args>(args)...);
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

    static boost::shared_ptr<T> m_pSington;
    static boost::mutex m_Mutex;
};

template <typename T>
boost::shared_ptr<T> Singleton<T>::m_pSington = nullptr;

template <typename T>
boost::mutex Singleton<T>::m_Mutex;


#endif