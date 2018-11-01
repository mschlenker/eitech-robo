#!/usr/bin/ruby

File.open(ARGV[0]).each { |line|
	unless line.strip =~ /^\/\// 
		(line + "\r").chars { |c|
			print "0x%02x, " % [ c.ord ]
		}
		print "\n"
	end
}
