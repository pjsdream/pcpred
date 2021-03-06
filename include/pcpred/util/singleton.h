#ifndef SINGLETON_H
#define SINGLETON_H


namespace pcpred
{

template<class T>
class Singleton
{
public:

	virtual ~Singleton(void) {}
    static bool isCreated();
	static T* getInstance();
    static void destroy();
    Singleton(void) {}

protected:

	static T* instance_;
};

template<class T>
T* Singleton<T>::instance_ = 0;

template<class T>
bool Singleton<T>::isCreated()
{
    return instance_ != 0;
}

template<class T>
T* Singleton<T>::getInstance()
{
    if (instance_ == 0)
		instance_ = new T;
	return instance_;
}

template<class T>
void Singleton<T>::destroy()
{
    delete instance_;
    instance_ = 0;
}

}


#endif // SINGLETON_H
