# Urho3D SoftBody
  
---
### Description
Bullet Physics SoftBody implementation for Urho3D. Originally, this implmentation was written just to satisfy my curiosity, and I posted it on the Urho3D forum on Aug., 2015. At the time, I was very new to the Urho3D engine and wasn't really knowledgeable about the engine or how to write components properly. But luckly, other members on the forum, specifcally, codingmonkey and Mike, completed missing elements in my implementation. This work is the result of our combined efforts.
  
#### Note
SoftBody nodes are in world space and the copy to the vertex buffer is not transformed into local space, and the reason the Urho3D Nodes are kept at zero position and rotation. You can change it to the typical format by setting model Node in world space and vertex buffer in local space, but I suspect there will be significant performance hit implementing this process.  If you'd like to tackle this and have questions on how to do it, post a message on the Urho3D discourse.

---
### Notable Settings
* ConfigPR - when a softbody encounters a high collision impact, there are times when all its face normals get reversed. To prevent it, set the **ConfigPR** with a high value, default=1.0.
* SetFaceNormal - apply face normal instead of average normal caused by duplicate vert removal, not applicable for Bullet generated mesh, default=false.

---
### To Build
To build it, unzip/drop the repository into your Urho3D/ folder and build it the same way as you'd build the default Samples that come with Urho3D.  
**Built with Urho3D 1.7 tag.**
  
---  
### License
The MIT License (MIT)







