#version 330 core
in vec2 uv;
uniform sampler2D hdrBuffer;
uniform sampler2D aoTexture;
uniform float exposure;
uniform int tonemapper;   // 0 = none (clamp), 1 = reinhard, 2 = ACES fitted
uniform int ssaoEnabled;
uniform float ssaoStrength;
uniform int aoDebug;   // 1 = show the ambient-occlusion buffer instead of the scene

// --- camera sensor -------------------------------------------------------
uniform sampler2D velocityTexture;
uniform int autoExposure;
uniform float exposureKey;       // target average luminance, 0.18 is mid grey
uniform vec2 exposureLimits;     // min and max EV the meter may choose
uniform sampler2D bloomTexture;
uniform float bloomStrength;
uniform float vignetteStrength;
uniform float noiseStrength;     // sensor noise, as a fraction of full scale
uniform float motionBlur;        // 0..1 of a frame's motion
uniform float frameSeed;
uniform vec3 whiteBalance;   // colour of the illuminant the camera is balanced for
uniform vec2 screenSize;
out vec4 fragColor;

// Narkowicz 2015 ACES filmic approximation.
vec3 aces(vec3 x)
{
	const float a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
	return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

vec3 linearToSrgb(vec3 c)
{
	return mix(c * 12.92, 1.055 * pow(c, vec3(1.0 / 2.4)) - 0.055, step(vec3(0.0031308), c));
}

void main(void)
{
	if (aoDebug == 1)
	{
		float ao = ssaoEnabled == 1 ? texture(aoTexture, uv).r : 1.0;
		fragColor = vec4(ao, ao, ao, 1.0);
		return;
	}
	vec4 scene = texture(hdrBuffer, uv);
	vec3 hdr = scene.rgb;

	// Motion blur: smear along this pixel's screen-space velocity, which is the same
	// buffer temporal anti-aliasing reprojects with. A real shutter integrates the
	// scene while it is open, and `motionBlur` is that opening as a fraction of the
	// frame time.
	if (motionBlur > 0.0)
	{
		vec2 velocity = texture(velocityTexture, uv).rg * motionBlur;
		if (dot(velocity, velocity) > 1e-9)
		{
			vec3 sum = hdr;
			for (int i = 1; i < 6; i++)
			{
				float t = float(i) / 5.0 - 0.5;
				sum += texture(hdrBuffer, uv - velocity * t).rgb;
			}
			hdr = sum / 6.0;
		}
	}
	if (ssaoEnabled == 1)
	{
		// scene.a is the ambient share of this pixel (see forward.frag): occlude only that.
		float ao = mix(1.0, texture(aoTexture, uv).r, ssaoStrength);
		hdr *= mix(1.0, ao, scene.a);
	}
	float appliedExposure = exposure;
	if (autoExposure == 1)
	{
		// Meter off the smallest mip, which is the average of the whole frame. A real
		// camera's auto-exposure does the same thing with far fewer samples.
		// Averaged from a small mip rather than the single top level, over a spread of
		// taps: the very top level of a non-square texture is not a reliable mean.
		float levels = max(floor(log2(max(screenSize.x, screenSize.y))) - 2.0, 1.0);
		vec3 average = vec3(0.0);
		for (int y = 0; y < 3; y++)
			for (int x = 0; x < 3; x++)
				average += textureLod(hdrBuffer,
									  vec2(float(x) + 0.5, float(y) + 0.5) / 3.0,
									  levels).rgb;
		average /= 9.0;
		float averageLuminance = max(dot(average, vec3(0.2126, 0.7152, 0.0722)), 1e-6);
		float metered = exposureKey / averageLuminance;
		float ev = -log2(max(metered, 1e-12) * 1.2);
		ev = clamp(ev, exposureLimits.x, exposureLimits.y);
		appliedExposure = 1.0 / (1.2 * exp2(ev));
	}

	// Bloom: bright parts of the image spill into their surroundings, the way light
	// scatters inside a real lens. Prepared in its own bright-pass and blur, because
	// thresholding after a blur produces a halo shaped like the blur kernel.
	if (bloomStrength > 0.0)
		hdr += texture(bloomTexture, uv).rgb * bloomStrength;

	// White balance. The lights are physical, so a 5500 K sun really is warmer than
	// sRGB's 6500 K white point; a camera cancels that by dividing through the colour
	// of the illuminant it is balanced for, and so do we. Balanced to the sun, a
	// sunlit surface comes out neutral and any *other* light source keeps its true
	// relative tint - a tungsten lamp still looks warm, the sky still looks blue.
	hdr /= max(whiteBalance, vec3(1e-4));

	hdr *= appliedExposure;
	vec3 mapped;
	if (tonemapper == 2)
		mapped = aces(hdr);
	else if (tonemapper == 1)
		mapped = hdr / (hdr + vec3(1.0));
	else
		mapped = clamp(hdr, 0.0, 1.0);
	vec3 output_ = linearToSrgb(mapped);

	// Vignette: every lens loses light towards the corners.
	if (vignetteStrength > 0.0)
	{
		vec2 centred = (uv - 0.5) * 2.0;
		float falloff = 1.0 - vignetteStrength * dot(centred, centred) * 0.25;
		output_ *= clamp(falloff, 0.0, 1.0);
	}

	// Sensor noise, scaled by the square root of the signal the way photon shot noise
	// is, so dark parts of the frame are the noisy ones - as on a real sensor.
	if (noiseStrength > 0.0)
	{
		vec3 seeded = vec3(uv, frameSeed);
		float n = fract(sin(dot(seeded, vec3(12.9898, 78.233, 37.719))) * 43758.5453);
		float luminance = dot(output_, vec3(0.2126, 0.7152, 0.0722));
		output_ += (n - 0.5) * noiseStrength * sqrt(max(luminance, 0.02));
	}

	fragColor = vec4(clamp(output_, 0.0, 1.0), 1.0);
}
