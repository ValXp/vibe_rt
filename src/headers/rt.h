#ifndef RT_H_
# define RT_H_

# include<string>
# include <cstdint>
#include <SFML/Graphics.hpp>

void init(unsigned int width, unsigned int height, std::string objPath);
void renderToTexture(std::uint8_t* pixels, float x, float y, float z);

#endif /* RT_H_ */
