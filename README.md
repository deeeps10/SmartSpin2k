# SmartSpin2k
This project focuses on reverse engineering 3D geometry from STL files to recreate accurate parametric models. The process begins by converting the STL mesh into a solid body in Autodesk Fusion 360, enabling access to parametric features. Key coordinates and dimensions of various sketch profiles and features—such as extrudes, cuts, and revolves—are then measured to replicate the original design. These feature details are fed into Antigravity/vscode and with the assistance of Claude, a build123d script is generated. Running this script produces a reconstructed 3D model that closely matches the original geometry. To validate accuracy, the volume of the generated model is compared with the original using code . the refernce and generated files are compared in the code itself.also another validation code is added that compares the stl files in the folder .

volumetric difference -

*Knob_Cup_V2-0.2%
*60mm-0.47%
*65mm-0.211
*plug-0.02%
*pooboo-0.0870%
*armwithhook20-0.04%
*armwithhook25-0.0047%
*armwithhook30-0.022%
*armwithhook35-0.026%
*armwithhook40-0.013%
*armwithhook45-0.013%
*50mm-0.068%
*armwithhook50-0.01%
*armwithhook55-0.012%
*armwithhook60-0.01%
*armwithhook65-0.01%
*armwithhook70-0.012%
*armwithhook75-0.01%
*armwithhook80-0.009%
*armwithhook85-0.008%
*armwithhook90-0.005%
*armwithhook95-0.0081%
*body_bike-0.29%
*JLL-IC400-0.27%
*Yesoul S3-0.12%
*retainer-0.034%
*window-0.012%
*armwithhook100-0.023%
*armwithhook105-0.004%
*armwithhook110-0.007%
*armwithhook115-0.006%
*armwithhook120-0.005%
*armwithhook125-0.003%
*armwithhook130-0.005%
*armwithhook135-0.004%
*armwithhook140-0.003%
*armwithhook145-0.005%
*armwithhook150-0.004%
*armwithhook155-0.003%
*bowflex-0.38%
*BH SB3 spinbike Insert v2-0.06%
*Mounting_StrapV2_XL-0.12%
*Mounting_StrapV2-0.07%
*Spur gear 1M 30T-0.23%
*Spur gear 1M 31T-0.025
*Spur gear 1M 40T Hex-0.22%
*Mounting_Strap_Rigid-0.088%
*Insert1-0.023%
*Insert2-0.13%
*Insert3-0.06%
*Mounting_For_New_Case-0.051%
*BH SB3 spinbike Insert v3-0.13%
*LoganRetentionClip-0.005%
*Revmaster-Spin_Adaptor-0.02%
*Spur gear 1M 11T-0.03%
*Spur gear 1M 11T_1-0.33%
*Spur gear 1M 11T-0.05%
*Schwinn_Insert-
*FLYWHEEL_54.5mm-
*bolt_arm-
*bolt through arm-
*BakerEchelonStrap-
