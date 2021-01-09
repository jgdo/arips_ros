#pragma once

#include <map>
#include <string>
#include <stdexcept>

namespace toponav_ros {

template <class T>
class NamedModuleContainer: public std::map<std::string, typename T::Ptr> {
public:
	typedef typename T::Ptr t_type;
	
	void addModule(std::string type, const t_type &plugin) {
		if (!plugin)
			throw std::runtime_error("NamedModuleContainer::addModule(\"" + type + "\"): plugin is null");
		
		if (this->find(type) != this->end())
			throw std::runtime_error(
					"NamedModuleContainer::addModule(): already have a plugin for type \"" +
					type + "\"");
		
		this->emplace(type, plugin);
	}
	
	t_type getModule(std::string type) {
		auto iter = this->find(type);
		if (iter == this->end())
			throw std::runtime_error(
					"NamedModuleContainer::getModule(): cannot find plugin for type \"" +
					type + "\"");
		
		return iter->second;
	}
	
	t_type getModuleOrNull(std::string type) {
		auto iter = this->find(type);
		if (iter != this->end())
			return iter->second;
		else
			return t_type();
	}
	
	const t_type getModuleOrNull(std::string type) const {
		auto iter = this->find(type);
		if (iter != this->end())
			return iter->second;
		else
			return t_type();
	}
};

} // namespace topo_nav
