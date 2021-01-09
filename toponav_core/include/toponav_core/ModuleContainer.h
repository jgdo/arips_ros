#pragma once

#include <map>
#include <typeindex>
#include <typeinfo>

#include <boost/any.hpp>

namespace toponav_core {

class ModuleContainer {
public:
	template<class T>
	typename T::Ptr getCreateModule() {
		auto iter = module_map_.find(toIndex<T>());
		if(iter == module_map_.end())
			throw std::runtime_error(std::string("ModuleContainer::getCreateModule(): No plugin for type ") + typeid(T).name() + " found");
		
		try {
			return boost::any_cast<typename T::Ptr>(module_map_[toIndex<T>()]);
		} catch(const boost::bad_any_cast& e) {
			throw std::runtime_error(std::string("ModuleContainer::getCreateModule(): Failed to retrieve plugin for type ") + typeid(T).name() + " found");
		}
	}

	template<class T>
	void addModule(typename T::Ptr const &plugin) {
		module_map_.emplace(toIndex<T>(), boost::any(plugin));
	}

private:
	std::map<std::type_index, boost::any> module_map_;

	template <class T>
	inline static std::type_index toIndex() {
		return std::type_index(typeid(T));
	}
};

} // namespace topo_nav

