# Intro
This project is a custom ray-traced renderer initially built as a way to get deeper into graphics programming. It's strongly stylized to have fuzzy dreamlike visuals, but this is also partially as a rendering technique to increase the number of samples per pixel while still feeling like it's in real-time. It's built using the Rust objc2 crate for bindings to Appkit and Foundation, as well as Metal for shader code. I referenced the blog post [Ray Tracing in One Weekend](https://raytracing.github.io/books/RayTracingInOneWeekend) frequently for the basics on ray-tracing, but implemented it GPU-sided instead. I have plans to extend the project into a full game with multiplayer support, but currently it's shelved for now.

Additional shout-outs to this blog on [bvh basics](https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/) which I referenced to implement my own BVH acceleration structures, as well as [Inigo Quilez](https://iquilezles.org/articles/simplepathtracing/) who I didn't directly reference but whose site has been incredibly useful while I continue to learn graphics programming. Also implemented Kruskal's algorithm thanks to [Jamis Buck](https://weblog.jamisbuck.org/2011/1/3/maze-generation-kruskal-s-algorithm) for randomized maze generation (I used to have a hard coded map before since I don't have a level editor, I do not recommend)

<img width="1014" alt="Image of a dark maze" src="https://github.com/user-attachments/assets/85036634-b56d-4107-b767-cc5ffa1beccd" />

# Installation
This currently only works on macOS, but I have plans to create a port for Windows at some point. Simply clone the repository and use **cargo run** to launch.
