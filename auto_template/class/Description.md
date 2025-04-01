[Templates](../TemplateCode.md)

A Template to use eros content to generate a C++ Class.

# Auto Code Generation
```bash
cd <eros>
cookiecutter -f auto_template/class -o <new directory>
```

# Integration Steps
In order to integrate this content into your package, do the following:
1. Move the generated content include into your package, typically under a folder `src`
2. Update the src CMakeLists file with adding your new Class folder.
3. Make the node your own!  Add whatever code you like.