# VS Code Setup
1. Install the Recommended VS Code extensions by opening the Extensions Sidebar and typing `@recommended`.
2. Setup pre-commit:
`pip install pre-commit`
`pre-commit install`

# Generate Doxygen HTML

```
git checkout master
cd /home/robot/catkin_ws/src/eros
doxygen Doxyfile
git commit --allow-empty -m "Trigger rebuild"
git push
```

# Generate Code Coverage
1. Build content and run unit tests: [Setup](../../README.md#setup-instructions).
2. Generate Code Coverage:
```code
cd <eros directory>
./scripts/dev_tools.sh code_coverage
```

# Template Code
Various Templates are provided to enable easy use of eROS content in your application.  See [Templates](../../auto_template/TemplateCode.md) for more information.