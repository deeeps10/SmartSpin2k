# SmartSpin2k
This project focuses on reverse engineering 3D geometry from STL files to recreate accurate parametric models. The process begins by converting the STL mesh into a solid body in Autodesk Fusion 360, enabling access to parametric features. Key coordinates and dimensions of various sketch profiles and features—such as extrudes, cuts, and revolves—are then measured to replicate the original design. These feature details are fed into Antigravity/vscode and with the assistance of Claude, a build123d script is generated. Running this script produces a reconstructed 3D model that closely matches the original geometry. To validate accuracy, the volume of the generated model is compared with the original using code . the refernce and generated files are compared in the code itself.also another validation code is added that compares the stl files in the folder .  
volumetric difference -
Knob_Cup_V2-0.2%
60mm-0.47%
