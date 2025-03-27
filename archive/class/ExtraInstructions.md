Now perform the following:
1. Move the generated content include into your package include path (Follow ROS conventions and move to include/<PackageName>/).
2. Move the generated content src into your package src path (Follow ROS conventions and move to src/<PackageName>).
3. Update the src/<PackageName>/CMakeLists.txt file with the generated src/<PackageName>/_CMakeLists.txt content.
4. Update the src/<PackageName>/<Class>/test/CMakeLists.txt file with the generated src/<PackageName>/<Class>/test/CMakeLists.txt content.
