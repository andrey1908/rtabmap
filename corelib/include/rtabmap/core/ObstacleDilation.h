#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/SemanticDilation.h>
#include <rtabmap/core/BaseClasses.h>

#include <memory>

namespace rtabmap {

class ObstacleDilation : BaseClasses
{
public:
	ObstacleDilation(const ParametersMap& parameters = ParametersMap());
	void parseParameters(const ParametersMap& parameters);

	std::shared_ptr<LocalMap> dilate(const LocalMap& localMap) const;

	float dilationSize() { return dilationSizeMeters_; }

private:
	float cellSize_;
	float dilationSizeMeters_;
	int dilationSize_;

	std::unique_ptr<SemanticDilation> semanticDilation_;
};

}
