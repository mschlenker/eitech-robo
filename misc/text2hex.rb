#!/usr/bin/ruby

puts "static const unsigned char index_html[] = {"

File.open(ARGV[0]).each { |line|
	unless line.strip =~ /^\/\// || line.strip == ""
		line = line.gsub("BUILDDATE", Time.now.utc.to_s)
		(line + "\r").chars { |c|
			print "0x%02x, " % [ c.ord ]
		}
		print "\n"
	end
}

puts "};"
