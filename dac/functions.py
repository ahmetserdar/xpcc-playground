
import math, argparse
import jinja2


def sine(pts, low=0, high=0xfff):
	omega     = 2 * math.pi / pts
	amplitude = high - low
	offset    = (high + low) / 2
	amplitude = amplitude / 2
	return [int(offset + amplitude * math.sin(omega * x)) for x in range(pts)]



CPP_LIST_TEMPLATE = """
static constexpr uint16_t {{name|upper}}_LENGTH = {{lst|length}};
static constexpr uint16_t {{name|upper}}[{{lst|length}}] = {
{% for l in lst -%}
{{l}},
{% endfor -%}
};
"""

def listToCppArray(name, lst):
	template = jinja2.Template(CPP_LIST_TEMPLATE)
	print template.render({'name': name, 'lst': lst})


if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Generate Data to be able to use the DAC as a function generator')
	parser.add_argument('-p', '--points', action='store', help='The number of points per function.', default=256)

	args = parser.parse_args()

	listToCppArray("sine", sine(int(args.points)))
