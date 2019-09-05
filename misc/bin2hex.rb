#!/usr/bin/ruby

puts "static const unsigned char " + ARGV[1] + "[] = { "

bcount = 0

File.open(ARGV[0]).each_byte { |b|
	print "0x%02x, " % [ b.ord ]
	bcount += 1
	print "\n" if (bcount % 20 == 0)
}

puts " };"
