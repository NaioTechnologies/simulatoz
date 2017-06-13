#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/rendering/rendering.hh"

#include <sstream>

namespace gazebo
{
class LeekPlanter 
	: public WorldPlugin
{
public:
	void Load(physics::WorldPtr parent, sdf::ElementPtr sdf )
	{
		size_t n = 0;
		sdf::ElementPtr leek = sdf->GetFirstElement();
		if( !leek ) return;

		do
		{
			if( leek->GetName() != "pose" ) continue;
			math::Vector2d xy = leek->Get<math::Vector2d>();	
			double x = xy.x;
			double y = xy.y;
			double z = 5;
			for( ; z >= -5; z -= 0.05 )
			{
				if( !parent->GetModelBelowPoint( math::Vector3( x, y, z ) ) )
				{
					break;
				} 
			}
			
			z += 0.1;
			std::ostringstream sdftext;
			sdftext << "<sdf version='1.6'>\n"
				<< "  <model name=\"leek_" << n << "\">"
				<< "  <pose>" << x << " " << y << " " << z << " 0 0 0</pose>\n" // -125.747567
				<< "  <include>"
				<< "	<uri>model://Leek</uri>\n"
				<< "	<static>true</static>\n"
				<< "  </include>"
				<< "</model>"
				<< "</sdf>";
			parent->InsertModelString(sdftext.str());
			n++;
		} while( leek = leek->GetNextElement() );
	}
};

GZ_REGISTER_WORLD_PLUGIN(LeekPlanter)
}

