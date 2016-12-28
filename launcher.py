import roslaunch

package = 'oz440_gazebo'
executable = 'level1_no_gfx.launch'
node = roslaunch.core.Node(package, executable)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
print "HELLO"
process.stop()
