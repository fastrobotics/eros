[Templates](../TemplateCode.md)

A Template to use eros content to generate a ROS Node.

# Auto Code Generation
```bash
cd <eros>
cookiecutter -f auto_template/node -o <new directory>
```

# Integration Steps
In order to integrate this content into your package, do the following:
1. Move the generated content include into your package, typically under a folder `nodes`
3. Update the nodes CMakeLists file with adding your new Node folder.
4. In the generated Node Class header, update the following entries: 
    * MAJOR_RELEASE_VERSION
    * MINOR_RELEASE_VERSION
    * BUILD_NUMBER
    * FIRMWARE_DESCRIPTION
5. In the generated Node Class cpp file, update the following entries:
    * diagnostic_types.push_back(...) // See [eros_Definitions.h](../../include/eros/eros_Definitions.h) for appropriate values.
6. In the generated Node Launch file, change the package name of the node to your package.
7. Make the node your own!  Add whatever code you like.