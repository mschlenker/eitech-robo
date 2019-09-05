#!/usr/bin/ruby
# encoding: utf-8

puts "static const unsigned char index_html[] = {"

File.open(ARGV[0]).each { |line|
	unless line.strip =~ /^\/\// || line.strip == ""
		line = line.gsub("BUILDDATE", Time.now.utc.to_s)
		# line = line.gsub("ä", "&auml;").gsub("ö", "&ouml;").gsub("ü", "&uuml;").gsub("Ä", "&Auml;").gsub("Ö", "&Ouml;").gsub("Ü", "&Uuml;").gsub("ß", "&szlig;")
		line.chars { |c|
			print "0x%02x, " % [ c.ord ]
		}
		print "\n"
	end
}

puts "};"
