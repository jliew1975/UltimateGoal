#!/usr/bin/perlml

## In "mailform.cgi" --
use CGI::Application::Mailform;

# Create a new Mailform instance...
my $mf = CGI::Application::Mailform->new();

# Configure your mailform
$mf->param('MAIL_FROM'   => 'webmaster@eborgrobotics.org');
$mf->param('MAIL_TO'     => 'john.liew@eborgrobotics.org');
$mf->param('HTMLFORM_REDIRECT_URL' => '../contact.html');
$mf->param('SUCCESS_REDIRECT_URL'  => '../contact.html');
$mf->param('FORM_FIELDS' => [qw/firstName lastName email phoneNo message/]);

# Optional variables
$mf->param('SUBJECT'     => 'New form submission');
$mf->param('ENV_FIELDS'  => [qw/REMOTE_ADDR HTTP_USER_AGENT/]);

# Now run...
$mf->run();
exit(0);