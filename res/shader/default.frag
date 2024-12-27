#version 330 core

in float angle;
in float distance;

out vec4 fragColor;

uniform vec3 color;

vec3 hueToRGB(float hue)
{
	hue *= 6.0;

	float r = abs(hue - 3.0) - 1.0;
	float g = 2.0 - abs(hue - 2.0);
	float b = 2.0 - abs(hue - 4.0);

	return vec3(min(1.0, max(0.0, r)), min(1.0, max(0.0, g)), min(1.0, max(0.0, b)));
}

void main()
{
	fragColor = vec4(1.0f, 1.0f, 1.0f, 1.0f);
	fragColor = vec4(color, 1.0);

	if(color.r > 0.5)
		fragColor = vec4(hueToRGB(angle / 360.0), 1.0);
	else
		fragColor = vec4(0.0);
}