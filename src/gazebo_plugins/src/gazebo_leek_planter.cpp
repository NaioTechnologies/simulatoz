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

		physics::ModelPtr model = parent->GetModel( "heightmap" );
		physics::HeightmapShape* hm = dynamic_cast<physics::HeightmapShape*>( model->GetLink("link")->GetCollision("collision")->GetShape().get() );
		math::Vector3 size = hm->GetSize(); 
		math::Vector2i vc = hm->GetVertexCount();
		double ss = static_cast<double>( hm->GetSubSampling() );

		do
		{
			std::string modelname;
			double zdiff;
			if( leek->GetName() == "leek" )
			{
				modelname = "Leek";
				zdiff = 0.1;
			}
			else if ( leek->GetName() == "redpost" )
			{
				modelname = "Red_stick";
				zdiff = 0.15;
			}
			else if ( leek->GetName() == "cabbage" )
			{
				modelname = "Cabbage";
				zdiff = 0.1;
			}
			else
			{
				continue;
			}
			math::Vector2d xy = leek->Get<math::Vector2d>();	
			double x = xy.x;
			double y = xy.y;
			double z = hm->GetHeight( (x + size.x/2)/size.x*vc.x - 1, (y + size.y/2)/size.y*vc.y - 1);
			z += zdiff;
			std::ostringstream sdftext;
			sdftext << "<sdf version='1.6'>\n"
				<< "  <model name=\"" << modelname << "_" << n << "\">"
				<< "    <pose>" << x << " " << y << " " << z << " 0 0 0</pose>\n" // -125.747567
				<< "    <include>"
				<< "	  <uri>model://" << modelname << "</uri>\n"
				<< "	  <static>true</static>\n"
				<< "    </include>"
				<< "  </model>"
				<< "</sdf>";
			parent->InsertModelString(sdftext.str());
			n++;
		} while( leek = leek->GetNextElement() );
	}
};

GZ_REGISTER_WORLD_PLUGIN(LeekPlanter)
}

